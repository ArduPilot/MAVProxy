#!/usr/bin/env python
'''
access satellite map tile database

some functions are based on code from mapUtils.py in gmapcatcher

Andrew Tridgell
May 2012
'''

import math, cv, sys, os, mp_util, numpy, httplib2, threading, time, collections, string, hashlib, errno

class TileException(Exception):
	'''tile error class'''
	def __init__(self, msg):
		Exception.__init__(self, msg)

TILE_SERVICES = {
	# thanks to http://go2log.com/2011/09/26/fetching-tiles-for-offline-map/
	# for the URL mapping info
	"GoogleSat"      : "http://khm${GOOG_DIGIT}.google.com/kh/v=58&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
	"GoogleMap"      : "http://mt${GOOG_DIGIT}.google.com/vt/lyrs=m@121&hl=en&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
	"GoogleHyb"      : "http://mt${GOOG_DIGIT}.google.com/vt/lyrs=h@121&hl=en&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
	"GoogleTer"      : "http://mt${GOOG_DIGIT}.google.com/vt/lyrs=t@108,r@121&hl=en&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
	"GoogleChina"    : "http://mt${GOOG_DIGIT}.google.cn/vt/lyrs=m@121&hl=en&gl=cn&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
	"YahooMap"       : "http://maps${Y_DIGIT}.yimg.com/hx/tl?v=4.3&.intl=en&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM}&r=1",
	"YahooSat"       : "http://maps${Y_DIGIT}.yimg.com/ae/ximg?v=1.9&t=a&s=256&.intl=en&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM}&r=1",
	"YahooInMap"     : "http://maps.yimg.com/hw/tile?locale=en&imgtype=png&yimgv=1.2&v=4.1&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM_2}",
	"YahooInHyb"     : "http://maps.yimg.com/hw/tile?imgtype=png&yimgv=0.95&t=h&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM_2}",
	"YahooHyb"       : "http://maps${Y_DIGIT}.yimg.com/hx/tl?v=4.3&t=h&.intl=en&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM}&r=1",
	"MicrosoftBrMap" : "http://imakm${MS_DIGITBR}.maplink3.com.br/maps.ashx?v=${QUAD}|t&call=2.2.4",
	"MicrosoftHyb"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/h${QUAD}.png?g=441&mkt=en-us&n=z",
	"MicrosoftSat"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/a${QUAD}.png?g=441&mkt=en-us&n=z",
	"MicrosoftMap"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&n=z",
	"MicrosoftTer"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&shading=hill&n=z",
	"OpenStreetMap"  : "http://tile.openstreetmap.org/${ZOOM}/${X}/${Y}.png",
	"OSMARender"     : "http://tah.openstreetmap.org/Tiles/tile/${ZOOM}/${X}/${Y}.png",
	"OpenAerialMap"  : "http://tile.openaerialmap.org/tiles/?v=mgm&layer=openaerialmap-900913&x=${X}&y=${Y}&zoom=${OAM_ZOOM}",
	"OpenCycleMap"   : "http://andy.sandbox.cloudmade.com/tiles/cycle/${ZOOM}/${X}/${Y}.png"
	}

# these are the md5sums of "unavailable" tiles
BLANK_TILES = set(["d16657bbee25d7f15c583f5c5bf23f50",
                   "c0e76e6e90ff881da047c15dbea380c7",
		   "d41d8cd98f00b204e9800998ecf8427e"])

class TileServiceInfo:
	'''a lookup object for the URL templates'''
	def __init__(self, x, y, zoom):
		self.X = x
		self.Y = y
		self.Z = zoom
		quadcode = ''
		for i in range(zoom - 1, -1, -1):
			quadcode += str((((((y >> i) & 1) << 1) + ((x >> i) & 1))))
		self.ZOOM = zoom
		self.QUAD = quadcode
		self.YAHOO_Y = 2**(zoom-1) - 1 - y
		self.YAHOO_ZOOM = zoom + 1
		self.YAHOO_ZOOM_2 = 17 - zoom + 1
		self.OAM_ZOOM = 17 - zoom
		self.GOOG_DIGIT = (x + y) & 3
		self.MS_DIGITBR = (((y & 1) << 1) + (x & 1)) + 1
		self.MS_DIGIT = (((y & 3) << 1) + (x & 1))
		self.Y_DIGIT = (x + y + zoom) % 3 + 1
		self.GALILEO = "Galileo"[0:(3 * x + y) & 7]

	def __getitem__(self, a):
		return str(getattr(self, a))

class MPTile:
	'''map tile object'''
	def __init__(self, cache_path=None, download=True, cache_size=500,
		     service="MicrosoftSat", tile_delay=1.0, debug=False,
		     max_zoom=19):
		if cache_path is None:
			cache_path = os.path.join(os.environ['HOME'], '.tilecache')
		self.cache_path = cache_path
		self.tiles_width = 256
		self.tiles_height = 256
		self.max_zoom = max_zoom
		self.min_zoom = 1
		self.download = download
		self.cache_size = cache_size
		self.tile_delay = tile_delay
		self.service = service
		self.debug = debug

		if service not in TILE_SERVICES:
			raise TileException('unknown tile service %s' % service)

		self._download_pending = []
		self._download_thread = None
		self._loading = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'data', 'loading.jpg')
		self._unavailable = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'data', 'unavailable.jpg')
		self._tile_cache = collections.OrderedDict()

	def coord_to_tile(self, lat, lon, zoom):
		'''convert lat/lon/zoom to tile
		returns ((tilex, tiley), (ofsx, ofsy))
		'''
		world_tiles = 1<<zoom
		x = world_tiles / 360.0 * (lon + 180.0)
		tiles_pre_radian = world_tiles / (2 * math.pi)
		e = math.sin(lat * (1/180.*math.pi))
		y = world_tiles/2 + 0.5*math.log((1+e)/(1-e)) * (-tiles_pre_radian)
		offsetx = int((x - int(x)) * self.tiles_width)
		offsety = int((y - int(y)) * self.tiles_height)
		return (int(x) % world_tiles, int(y) % world_tiles), (offsetx, offsety)

	def tile_to_coord(self, tile, offset, zoom):
		'''convert (tilex/tiley) and zoom to (lat, lon)'''
		(tilex, tiley) = tile
		(offsetx, offsety) = offset
		world_tiles = 1<<zoom
		x = ( tilex + 1.0*offsetx/self.tiles_width ) / (world_tiles/2.) - 1
		y = ( tiley + 1.0*offsety/self.tiles_height) / (world_tiles/2.) - 1
		lon = x * 180.0
		y = math.exp(-y*2*math.pi)
		e = (y-1)/(y+1)
		lat = 180.0/math.pi * math.asin(e)
		return (lat, lon)

	def tile_size(self, tile, zoom):
		'''return tile size as (width,height) in meters'''
		(lat1, lon1) = self.tile_to_coord(tile, (0,0), zoom)
		(lat2, lon2) = self.tile_to_coord(tile, (self.tiles_width,0), zoom)
		width = mp_util.gps_distance(lat1, lon1, lat2, lon2)
		(lat2, lon2) = self.tile_to_coord(tile, (0,self.tiles_height), zoom)
		height = mp_util.gps_distance(lat1, lon1, lat2, lon2)
		return (width,height)

	def tile_to_path(self, tile, zoom):
		'''convert a (tilex,tiley) to a file path'''
		(tilex, tiley) = tile
		return os.path.join(self.cache_path, "%s/%u/%u/%u.img" % (self.service, zoom, tiley, tilex))

	def coord_to_tilepath(self, lat, lon, zoom):
		'''return the tile ID that covers a latitude/longitude at
		a specified zoom level
		'''
		(tile,offset) = self.coord_to_tile(lat, lon, zoom)
		return self.tile_to_path(tile, zoom)

	def tiles_pending(self):
		'''return number of tiles pending download'''
		return len(self._download_pending)

	def _tile_key(self, tile, zoom):
		'''tile cache key'''
		return (tile, zoom)


	def downloader(self):
		'''the download thread'''
		http = httplib2.Http()
		while self.tiles_pending() > 0:
			time.sleep(self.tile_delay)
			idx = len(self._download_pending)-1
			obj = self._download_pending[idx]
			(url, path, key) = obj
			try:
				if self.debug:
					print("Downloading %s [%u left]" % (url, self.tiles_pending()))
				resp,img = http.request(url)
			except IOError as e:
				#print('Error loading %s' % url)
				self._tile_cache[key] = self._unavailable
				self._download_pending.pop(idx)
				if self.debug:
					print("Failed %s: %s" % (url, str(e)))
				continue

			h = open(path+'.tmp','w')

			# see if its a blank/unavailable tile
			md5 = hashlib.md5(img).hexdigest()
			if md5 in BLANK_TILES:
				if self.debug:
					print("blank tile %s" % url)
				self._tile_cache[key] = self._unavailable
				self._download_pending.pop(idx)
				continue

			h.write(img)
			h.close()
			os.rename(path+'.tmp', path)
			self._download_pending.pop(idx)
		self._download_thread = None

	def start_download_thread(self):
		'''start the downloader'''
		if self._download_thread:
			return
		t = threading.Thread(target=self.downloader)
		t.daemon = True
		self._download_thread = t
		t.start()

	def tile_url(self, x, y, zoom):
		'''return URL for a tile'''
		url = string.Template(TILE_SERVICES[self.service])
		tile_info = TileServiceInfo(x, y, zoom)
		return url.substitute(tile_info)


	def load_tile_lowres(self, tile, zoom):
		'''load a lower resolution tile from cache to fill in a
		map while waiting for a higher resolution tile'''
		if zoom == self.min_zoom:
			return None

		# find the equivalent lower res tile
		(lat,lon) = self.tile_to_coord(tile, (0,0), zoom)

		width2 = self.tiles_width
		height2 = self.tiles_height

		for zoom2 in range(zoom-1, self.min_zoom-1, -1):
			width2 /= 2
			height2 /= 2
			
			(tile, ofs) = self.coord_to_tile(lat, lon, zoom2)

			# see if its in the tile cache
			key = self._tile_key(tile, zoom2)
			if key in self._tile_cache:
				img = self._tile_cache[key]
				if img == self._unavailable:
					continue
			else:
				path = self.tile_to_path(tile, zoom2)
				try:
					img = cv.LoadImage(path)
					# add it to the tile cache
					self._tile_cache[key] = img
					while len(self._tile_cache) > self.cache_size:
						self._tile_cache.popitem(0)
				except IOError as e:
					continue

			cv.SetImageROI(img, (ofs[0], ofs[1], width2, height2))
			img2 = cv.CreateMat(height2, width2, cv.CV_8UC3)
			cv.Copy(img, img2)
			cv.ResetImageROI(img)
			scaled = cv.CreateMat(self.tiles_height, self.tiles_width, cv.CV_8UC3)
			cv.Resize(img2, scaled)
			#cv.Rectangle(scaled, (0,0), (255,255), (0,255,0), 1)
			return scaled
		return None

	def load_tile(self, tile, zoom):
		'''load a tile from cache or tile server'''
		(x,y) = tile

		# see if its in the tile cache
		key = self._tile_key(tile, zoom)
		if key in self._tile_cache:
			img = self._tile_cache[key]
			if img == self._unavailable:
				img = self.load_tile_lowres(tile, zoom)
				if img is None:
					img = cv.LoadImage(self._unavailable)
				return img			
				

		path = self.tile_to_path((x,y), zoom)
		try:
			ret = cv.LoadImage(path)
			# add it to the tile cache
			self._tile_cache[key] = ret
			while len(self._tile_cache) > self.cache_size:
				self._tile_cache.popitem(0)
			return ret
		except IOError as e:
			if not e.errno in [errno.ENOENT]:
				raise
			pass
		if not self.download:
			img = self.load_tile_lowres(tile, zoom)
			if img is None:
				img = cv.LoadImage(self._unavailable)
			return img			

		url = self.tile_url(x,y,zoom)
		mp_util.mkdir_p(os.path.dirname(path))
		pending_key = (url, path, key)
		if pending_key not in self._download_pending:
			self._download_pending.append(pending_key)
		self.start_download_thread()

		img = self.load_tile_lowres(tile, zoom)
		if img is None:
			img = cv.LoadImage(self._loading)
		return img
	

	def scaled_tile(self, tile, zoom, scale):
		'''return a scaled tile'''
		width = int(self.tiles_width / scale)
		height = int(self.tiles_height / scale)
		scaled_tile = cv.CreateMat(height, width, cv.CV_8UC3)
		full_tile = self.load_tile(tile, zoom)
		cv.Resize(full_tile, scaled_tile)
		return scaled_tile


	def coord_from_area(self, x, y, lat, lon, width, ground_width):
		'''return (lat,lon) for a pixel in an area image'''

		pixel_width = ground_width / float(width)
		dx = x * pixel_width
		dy = y * pixel_width

		return mp_util.gps_offset(lat, lon, dx, -dy)


	def area_to_tile_list(self, lat, lon, width, height, ground_width, zoom=None):
		'''return a list of tiles needed for an area of land, with ground_width 
		in meters, and width/height in pixels.

		lat/lon is the top left corner. If unspecified, the
		zoom is automatically chosen to avoid having to grow
		the tiles
		'''

		pixel_width = ground_width / float(width)
		ground_height = ground_width * (height/(float(width)))
		top_right = mp_util.gps_newpos(lat, lon, 90, ground_width)
		bottom_left = mp_util.gps_newpos(lat, lon, 180, ground_height)
		bottom_right = mp_util.gps_newpos(bottom_left[0], bottom_left[1], 90, ground_width)

		# choose a zoom level if not provided
		if zoom is None:
			zooms = range(self.min_zoom, self.max_zoom+1)
		else:
			zooms = [zoom]
		for zoom in zooms:
			(tile_min, ofs1) = self.coord_to_tile(lat, lon, zoom)
			(twidth,theight) = self.tile_size(tile_min, zoom)
			tile_pixel_width = twidth / float(self.tiles_width)
			scale = pixel_width / tile_pixel_width
			if scale >= 1.0:
				break

		scaled_tile_width = int(self.tiles_width / scale)
		scaled_tile_height = int(self.tiles_height / scale)

		# work out the bottom right tile
		(tile_max, ofs2) = self.coord_to_tile(bottom_right[0], bottom_right[1], zoom)

		(twidth,theight) = self.tile_size(tile_min, zoom)

		ofsx = int(ofs1[0] / scale)
		ofsy = int(ofs1[1] / scale)
		srcy = ofsy
		dsty = 0

		ret = []

		# place the tiles
		for y in range(tile_min[1], tile_max[1]+1):
			srcx = ofsx
			dstx = 0
			for x in range(tile_min[0], tile_max[0]+1):
				if dstx < width and dsty < height:
					ret.append(((x,y), zoom, scale, (srcx,srcy), (dstx,dsty)))
				dstx += scaled_tile_width-srcx
				srcx = 0
			dsty += scaled_tile_height-srcy
			srcy = 0
		return ret

	def area_to_image(self, lat, lon, width, height, ground_width, zoom=None):
		'''return an image for an area of land, with ground_width 
		in meters, and width/height in pixels.

		lat/lon is the top left corner. The zoom is automatically
		chosen to avoid having to grow the tiles'''

		# make a numpy array for the image
		img = numpy.zeros((height, width, 3), numpy.uint8)

		tlist = self.area_to_tile_list(lat, lon, width, height, ground_width, zoom)
		for t in tlist:
			(tile, zoom, scale, src, dst) = t
			scaled_tile = self.scaled_tile(tile, zoom, scale)
			stile = numpy.asarray(scaled_tile)
			scaled_tile_width = int(self.tiles_width / scale)
			scaled_tile_height = int(self.tiles_height / scale)
			partial_tile = stile[src[1]:min(scaled_tile_height, src[1]+height-dst[1]),
					     src[0]:min(scaled_tile_width, src[0]+width-dst[0])]
			img[dst[1]:dst[1]+partial_tile.shape[0],
			    dst[0]:dst[0]+partial_tile.shape[1]] = partial_tile

		# return as an RGB image
		img = cv.fromarray(img)
		cv.CvtColor(img, img, cv.CV_BGR2RGB)
		return img



if __name__ == "__main__":

	from optparse import OptionParser
	parser = OptionParser("mp_tile.py [options]")
	parser.add_option("--lat", type='float', default=-35.362938, help="start latitude")
	parser.add_option("--lon", type='float', default=149.165085, help="start longitude")
	parser.add_option("--width", type='float', default=1000.0, help="width in meters")
	parser.add_option("--service", default="YahooSat", help="tile service")
	parser.add_option("--zoom", default=None, type='int', help="zoom level")
	parser.add_option("--max-zoom", type='int', default=19, help="maximum tile zoom")
	parser.add_option("--delay", type='float', default=1.0, help="tile download delay")
	parser.add_option("--debug", action='store_true', default=False, help="show debug info")
	(opts, args) = parser.parse_args()

	mt = MPTile(debug=opts.debug, service=opts.service,
		    tile_delay=opts.delay, max_zoom=opts.max_zoom)
	if opts.zoom is None:
		zooms = range(mt.min_zoom, mt.max_zoom+1)
	else:
		zooms = [opts.zoom]
	for zoom in zooms:
		print("Using zoom %u" % zoom)
		tlist = mt.area_to_tile_list(opts.lat, opts.lon, width=12, height=512,
					     ground_width=opts.width, zoom=zoom)
		for t in tlist:
			(tile, zoom, scale, src, dst) = t
			mt.load_tile(tile, zoom)

	while mt.tiles_pending() > 0:
		time.sleep(2)
		print("Waiting on %u tiles" % mt.tiles_pending())
	print('Done')
