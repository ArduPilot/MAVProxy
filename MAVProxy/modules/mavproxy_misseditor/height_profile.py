#!/usr/bin/env python3
'''
Mission height profile window for the mission editor.

Shows mission height and terrain height along the mission track, plus a
height above ground pane, for checking terrain clearance when planning long
BVLOS missions.
'''

# AP_FLAKE8_CLEAN

import bisect
import math

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_elevation
from ..lib.wx_loader import wx

# terrain databases offered in the window
TERRAIN_SOURCES = ['SRTM1', 'SRTM3']

# nominal ground sample spacing of each database in metres. Terrain is
# sampled along the track at this spacing so the profile shows the detail
# the database actually holds
SOURCE_SPACING = {'SRTM1': 30.0, 'SRTM3': 90.0}

# altitude references the profile can be drawn against
ALT_REFERENCES = ['AMSL', 'AGL', 'AboveHome']

# height units, named as MAVProxy's height_unit setting does
HEIGHT_UNITS = ['m', 'feet']
FEET_PER_METRE = 3.28084

# cap on terrain samples for a whole mission, so that a very long BVLOS
# mission does not turn into a huge number of elevation lookups
MAX_SAMPLES = 4000

# budget on the total number of elevation lookups for one redraw. A wide
# corridor multiplies out quickly, so past this the corridor is sampled more
# coarsely in both directions rather than dropping cross track resolution,
# which would step over a narrow ridge
MAX_LOOKUPS = 400000

# largest profile width offered, in metres either side of the track
MAX_PROFILE_WIDTH = 5000

# minimum spacing between waypoint labels as a fraction of the track length,
# so that closely spaced waypoints do not print on top of each other
LABEL_SEPARATION = 0.012

NAN = float('nan')


class ProfilePoint(object):
    '''one mission item reduced to what the profile needs. The altitude is
       kept in its mission frame ("Abs", "Rel" or "AGL") so that it can be
       resolved against whichever terrain database is being shown'''

    def __init__(self, seq, lat, lon, alt, frame):
        self.seq = seq
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.frame = frame

    def is_terrain_frame(self):
        return self.frame == "AGL"


class HeightProfileFrame(wx.Frame):
    '''height profile of a mission against terrain'''

    def __init__(self, parent, elevation_models, source='SRTM3'):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Mission Height Profile",
                          size=(900, 700))
        # shared with the mission editor so tiles are only fetched once
        self.elevation_models = elevation_models
        self.source = source if source in TERRAIN_SOURCES else 'SRTM3'
        self.reference = 'AMSL'
        self.height_unit = 'm'
        # distance either side of the track that the worst case terrain is
        # taken over, so the corridor is twice this wide. Zero means sample
        # the track centreline only
        self.profile_width = 0.0
        # spacing of the cross track samples, set for each redraw
        self.cross_spacing = SOURCE_SPACING['SRTM3']
        self.subsampled = False
        self.points = []
        self.home_amsl = 0.0
        self.terrain_incomplete = False

        # sample arrays kept from the last draw, for the cursor readout
        self.plot_xs = []
        self.plot_vehicle = []
        self.plot_terrain = []
        self.plot_agl = []

        self.create_main_panel()
        self.Bind(wx.EVT_CLOSE, self.on_close)

    def create_main_panel(self):
        import platform
        if platform.system() == 'Darwin':
            from MAVProxy.modules.lib.MacOS import backend_wxagg
            FigCanvas = backend_wxagg.FigureCanvasWxAgg
        else:
            from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigCanvas
        from matplotlib.figure import Figure

        self.panel = wx.Panel(self)

        self.fig = Figure((8.0, 6.0), dpi=100)
        self.axes_height = self.fig.add_subplot(2, 1, 1)
        self.axes_agl = self.fig.add_subplot(2, 1, 2, sharex=self.axes_height)
        self.fig.subplots_adjust(hspace=0.25, left=0.1, right=0.97,
                                 top=0.95, bottom=0.09)
        self.canvas = FigCanvas(self.panel, -1, self.fig)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas.mpl_connect('axes_leave_event', self.on_mouse_leave)

        self.label_source = wx.StaticText(self.panel, wx.ID_ANY, "Terrain")
        self.choice_source = wx.Choice(self.panel, wx.ID_ANY,
                                       choices=TERRAIN_SOURCES)
        self.choice_source.SetSelection(TERRAIN_SOURCES.index(self.source))

        self.label_reference = wx.StaticText(self.panel, wx.ID_ANY, "Height")
        self.choice_reference = wx.Choice(self.panel, wx.ID_ANY,
                                          choices=ALT_REFERENCES)
        self.choice_reference.SetSelection(ALT_REFERENCES.index(self.reference))

        self.label_units = wx.StaticText(self.panel, wx.ID_ANY, "Units")
        self.choice_units = wx.Choice(self.panel, wx.ID_ANY, choices=HEIGHT_UNITS)
        self.choice_units.SetSelection(HEIGHT_UNITS.index(self.height_unit))

        self.label_width = wx.StaticText(self.panel, wx.ID_ANY, "Width (+-m)")
        self.spin_width = wx.SpinCtrl(self.panel, wx.ID_ANY, min=0,
                                      max=MAX_PROFILE_WIDTH,
                                      initial=int(self.profile_width))
        self.spin_width.SetToolTip(
            "Half width of the corridor, in metres either side of the track.\n"
            "50 means the worst case is taken over everything within 50m of\n"
            "the track, a corridor 100m across. Terrain height is the highest\n"
            "in the corridor and height above ground the lowest, so the\n"
            "profile shows the worst case if the mission is not flown exactly\n"
            "on track. Zero samples the track centreline only.")

        self.label_status = wx.StaticText(self.panel, wx.ID_ANY, "")

        self.Bind(wx.EVT_CHOICE, self.on_source_change, self.choice_source)
        self.Bind(wx.EVT_CHOICE, self.on_reference_change, self.choice_reference)
        self.Bind(wx.EVT_CHOICE, self.on_units_change, self.choice_units)
        self.Bind(wx.EVT_SPINCTRL, self.on_width_change, self.spin_width)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_width_change, self.spin_width)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(self.label_source, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
        hbox.Add(self.choice_source, 0, wx.ALL, 5)
        hbox.Add((20, 0), 0, 0, 0)
        hbox.Add(self.label_reference, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
        hbox.Add(self.choice_reference, 0, wx.ALL, 5)
        hbox.Add((20, 0), 0, 0, 0)
        hbox.Add(self.label_units, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
        hbox.Add(self.choice_units, 0, wx.ALL, 5)
        hbox.Add((20, 0), 0, 0, 0)
        hbox.Add(self.label_width, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)
        hbox.Add(self.spin_width, 0, wx.ALL, 5)
        hbox.Add((20, 0), 0, 0, 0)
        hbox.Add(self.label_status, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 5)

        vbox = wx.BoxSizer(wx.VERTICAL)
        vbox.Add(self.canvas, 1, flag=wx.EXPAND | wx.ALL)
        vbox.Add(hbox, 0, flag=wx.EXPAND)
        self.panel.SetSizer(vbox)

    def on_close(self, event):
        # the mission editor owns us, let it forget about us first
        parent = self.GetParent()
        if parent is not None:
            parent.height_profile_closed()
        event.Skip()

    def on_source_change(self, event):
        self.source = TERRAIN_SOURCES[self.choice_source.GetSelection()]
        self.draw_profile()

    def on_reference_change(self, event):
        self.reference = ALT_REFERENCES[self.choice_reference.GetSelection()]
        self.draw_profile()

    def on_units_change(self, event):
        self.height_unit = HEIGHT_UNITS[self.choice_units.GetSelection()]
        self.draw_profile()

    def on_width_change(self, event):
        self.profile_width = float(self.spin_width.GetValue())
        self.draw_profile()

    def cross_offsets(self):
        '''cross track offsets in metres to sample. profile_width is the
           distance either side of the track, and the corridor is stepped at
           the same resolution as the track itself so that widening it never
           steps over a ridge'''
        half = self.profile_width
        if half <= 0:
            return [0.0]
        steps = max(1, int(math.ceil(half / self.cross_spacing)))
        step = half / steps
        return [i * step for i in range(-steps, steps+1)]

    def worst_terrain(self, model, lat, lon, bearing):
        '''highest terrain in the corridor across the track at this point,
           which is the worst case for clearance. Returns None if any part
           of the corridor has no terrain, as the worst case is not known'''
        offsets = self.cross_offsets()
        if len(offsets) == 1:
            return model.GetElevation(lat, lon)
        worst = None
        for offset in offsets:
            if offset == 0:
                (slat, slon) = (lat, lon)
            else:
                side = bearing + (90 if offset > 0 else -90)
                (slat, slon) = mp_util.gps_newpos(lat, lon, side, abs(offset))
            elevation = model.GetElevation(slat, slon)
            if elevation is None:
                return None
            if worst is None or elevation > worst:
                worst = elevation
        return worst

    def height_convert(self, val_metres):
        '''convert a height in metres to the selected units'''
        if val_metres is None:
            return None
        if self.height_unit == 'feet':
            return val_metres * FEET_PER_METRE
        return val_metres

    def units_label(self):
        return 'ft' if self.height_unit == 'feet' else 'm'

    def elevation_model(self):
        '''elevation model for the selected source, created on first use'''
        if self.source not in self.elevation_models:
            self.elevation_models[self.source] = mp_elevation.ElevationModel(
                database=self.source)
        return self.elevation_models[self.source]

    def set_mission(self, points, home_amsl):
        '''give the window a new mission to show'''
        self.points = points
        self.home_amsl = home_amsl
        self.draw_profile()

    def needs_redraw(self):
        '''true if terrain was missing last time, so a retry is worthwhile
           once the downloader has had a chance to fetch the tiles'''
        return self.terrain_incomplete

    def leg_lon(self, lon1, lon2):
        '''return lon2 shifted so that the leg from lon1 takes the short way
           around. The rhumb line helpers in mp_util do not wrap the
           longitude difference, so a leg over the antimeridian would
           otherwise be measured the long way around the world'''
        delta = lon2 - lon1
        if delta > 180:
            return lon2 - 360
        if delta < -180:
            return lon2 + 360
        return lon2

    def leg_distance(self, prev, p):
        return mp_util.gps_distance(prev.lat, prev.lon, p.lat,
                                    self.leg_lon(prev.lon, p.lon))

    def leg_bearing(self, prev, p):
        return mp_util.gps_bearing(prev.lat, prev.lon, p.lat,
                                   self.leg_lon(prev.lon, p.lon))

    def sample_spacing(self, points):
        '''sample spacing in metres along the track and across the corridor.

           Both start at the resolution of the terrain database. A long
           mission, a wide corridor, or the two together can need more
           elevation lookups than is reasonable for one redraw, in which case
           both are coarsened together rather than dropping the cross track
           resolution alone, as that would step over a narrow ridge'''
        spacing = SOURCE_SPACING.get(self.source, 90.0)
        total = 0.0
        for i in range(1, len(points)):
            total += self.leg_distance(points[i-1], points[i])

        along = max(1.0, total / spacing)
        across = 2.0 * math.ceil(self.profile_width / spacing) + 1
        lookups = along * across
        self.subsampled = False
        if lookups > MAX_LOOKUPS:
            # scale both directions so the total lands within the budget
            spacing *= math.sqrt(lookups / MAX_LOOKUPS)
            self.subsampled = True
        elif total > 0 and total / spacing > MAX_SAMPLES:
            spacing = total / MAX_SAMPLES
            self.subsampled = True
        return spacing

    def point_amsl(self, point, ground):
        '''AMSL height of a mission item given the terrain under it'''
        if point.is_terrain_frame():
            if ground is None:
                return NAN
            return point.alt + ground
        if point.frame == "Rel":
            return point.alt + self.home_amsl
        return point.alt

    def sample_track(self):
        '''walk the mission track and return the samples along it. Terrain
           entries are None where the database has no tile yet.

           A leg into a terrain frame waypoint follows the terrain, as
           ArduPilot interpolates height above ground rather than AMSL for
           those legs, so a pair of AGL waypoints either side of a hill
           stays clear of it.

           With a profile width set, the terrain reported is the highest in
           a corridor of that width about the track, which is the worst case
           for clearance. A terrain following vehicle flies off the terrain
           beneath it, so the centreline terrain still places the vehicle'''
        points = self.points
        model = self.elevation_model()
        spacing = self.sample_spacing(points)
        # the corridor is stepped at the same resolution as the track
        self.cross_spacing = spacing

        dists = []
        terrain = []
        vehicle = []
        wp_index = []
        travelled = 0.0

        def add_sample(dist, ground, amsl):
            dists.append(dist)
            terrain.append(ground)
            vehicle.append(amsl)

        def ground_at(lat, lon, bearing):
            '''centreline terrain, and the worst in the corridor'''
            centre = model.GetElevation(lat, lon)
            if self.profile_width <= 0:
                return (centre, centre)
            return (centre, self.worst_terrain(model, lat, lon, bearing))

        # bearing at each waypoint, used for the corridor across it
        bearings = []
        for i in range(len(points)):
            if i > 0 and self.leg_distance(points[i-1], points[i]) > 0:
                bearings.append(self.leg_bearing(points[i-1], points[i]))
            elif i+1 < len(points) and self.leg_distance(points[i], points[i+1]) > 0:
                bearings.append(self.leg_bearing(points[i], points[i+1]))
            else:
                bearings.append(0.0)

        prev_ground = None
        for i in range(len(points)):
            p = points[i]
            (ground, ground_worst) = ground_at(p.lat, p.lon, bearings[i])
            if i > 0:
                prev = points[i-1]
                leg = self.leg_distance(prev, p)
                if leg > 0:
                    bearing = self.leg_bearing(prev, p)
                    prev_amsl = self.point_amsl(prev, prev_ground)
                    if p.is_terrain_frame():
                        # height above ground at the start of the leg, so a
                        # climb or descent to the new AGL height is spread
                        # along the leg the way the vehicle flies it
                        if prev_ground is None or prev_amsl != prev_amsl:
                            start_agl = p.alt
                        else:
                            start_agl = prev_amsl - prev_ground
                    else:
                        end_amsl = self.point_amsl(p, ground)
                    along = spacing
                    while along < leg:
                        (slat, slon) = mp_util.gps_newpos(prev.lat, prev.lon,
                                                          bearing, along)
                        (sground, sworst) = ground_at(slat, slon, bearing)
                        ratio = along / leg
                        if p.is_terrain_frame():
                            if sground is None:
                                samsl = NAN
                            else:
                                samsl = sground + start_agl + (p.alt - start_agl) * ratio
                        else:
                            samsl = prev_amsl + (end_amsl - prev_amsl) * ratio
                        add_sample(travelled + along, sworst, samsl)
                        along += spacing
                travelled += leg
            # remember where this waypoint landed so it can be labelled
            wp_index.append(len(dists))
            add_sample(travelled, ground_worst, self.point_amsl(p, ground))
            prev_ground = ground

        # a NaN vehicle height means an AGL item whose terrain is not in yet
        self.terrain_incomplete = (None in terrain or
                                   any(v != v for v in vehicle))
        return (dists, terrain, vehicle, wp_index)

    def make_readouts(self):
        '''create the cursor readout text and marker line on each pane'''
        self.readouts = []
        self.cursor_lines = []
        for axes in (self.axes_height, self.axes_agl):
            text = axes.text(0.995, 0.97, "", transform=axes.transAxes,
                             horizontalalignment='right', verticalalignment='top',
                             fontsize=8, family='monospace', zorder=5,
                             bbox=dict(boxstyle='round', facecolor='w',
                                       edgecolor='0.7', alpha=0.85))
            text.set_visible(False)
            self.readouts.append(text)
            line = axes.axvline(0, color='0.3', linewidth=0.8, zorder=4)
            line.set_visible(False)
            self.cursor_lines.append(line)

    def on_mouse_leave(self, event):
        self.hide_readouts()

    def hide_readouts(self):
        changed = False
        for artist in self.readouts + self.cursor_lines:
            if artist.get_visible():
                artist.set_visible(False)
                changed = True
        if changed:
            self.canvas.draw_idle()

    def format_value(self, value):
        '''format a sample value, already in the selected units'''
        if value is None or value != value:
            return "--"
        return "%.0f%s" % (value, self.units_label())

    def on_mouse_move(self, event):
        '''show the values under the cursor at the top right of each pane'''
        if event.inaxes not in (self.axes_height, self.axes_agl):
            self.hide_readouts()
            return
        if event.xdata is None or not self.plot_xs:
            self.hide_readouts()
            return

        # nearest sample to the cursor
        i = bisect.bisect_left(self.plot_xs, event.xdata)
        if i >= len(self.plot_xs):
            i = len(self.plot_xs) - 1
        elif i > 0:
            if event.xdata - self.plot_xs[i-1] < self.plot_xs[i] - event.xdata:
                i -= 1

        x = self.plot_xs[i]
        self.readouts[0].set_text(
            "%.2fkm\nmission %s\nterrain %s" % (
                x, self.format_value(self.plot_vehicle[i]),
                self.format_value(self.plot_terrain[i])))
        self.readouts[1].set_text(
            "%.2fkm\nAGL %s" % (x, self.format_value(self.plot_agl[i])))
        for artist in self.readouts:
            artist.set_visible(True)
        for line in self.cursor_lines:
            line.set_xdata([x, x])
            line.set_visible(True)
        self.canvas.draw_idle()

    def offset_for(self, terrain_amsl):
        '''height to subtract from an AMSL value for the current reference'''
        if self.reference == 'AboveHome':
            return self.home_amsl
        if self.reference == 'AGL':
            return terrain_amsl
        return 0.0

    def draw_profile(self):
        '''redraw both panes'''
        self.axes_height.clear()
        self.axes_agl.clear()

        if len(self.points) < 2:
            self.axes_height.set_title("mission has no track to profile")
            self.label_status.SetLabel("")
            self.plot_xs = []
            self.plot_vehicle = []
            self.plot_terrain = []
            self.plot_agl = []
            # nothing to wait on, so stop the terrain retry
            self.terrain_incomplete = False
            self.make_readouts()
            self.canvas.draw()
            return

        (dists, terrain, vehicle, wp_index) = self.sample_track()

        # x axis in km reads better over a long BVLOS mission
        xs = [d * 0.001 for d in dists]

        veh_plot = []
        ter_plot = []
        agl_plot = []
        for i in range(len(dists)):
            ground = terrain[i]
            if ground is None:
                # no tile yet, leave a gap rather than draw a wrong height
                veh_plot.append(NAN)
                ter_plot.append(NAN)
                agl_plot.append(NAN)
                continue
            offset = self.offset_for(ground)
            veh_plot.append(self.height_convert(vehicle[i] - offset))
            ter_plot.append(self.height_convert(ground - offset))
            agl_plot.append(self.height_convert(vehicle[i] - ground))

        self.axes_height.fill_between(xs, ter_plot, color='0.85')
        self.axes_height.plot(xs, ter_plot, color='0.4', linewidth=1,
                              label='terrain')
        self.axes_height.plot(xs, veh_plot, color='b', linewidth=1.5,
                              label='mission')

        # label waypoints on the height pane, dropping labels that would sit
        # on top of each other on a long mission
        min_sep = (xs[-1] - xs[0]) * LABEL_SEPARATION if len(xs) > 1 else 0
        last_label_x = None
        for (i, idx) in enumerate(wp_index):
            if idx >= len(xs) or veh_plot[idx] != veh_plot[idx]:
                # no terrain here, so no reference height to label against
                continue
            x = xs[idx]
            is_end = (i == 0 or i == len(wp_index)-1)
            if not is_end and last_label_x is not None and x - last_label_x < min_sep:
                continue
            last_label_x = x
            self.axes_height.annotate(str(self.points[i].seq),
                                      xy=(x, veh_plot[idx]),
                                      xytext=(0, 6), textcoords='offset points',
                                      fontsize=8, color='b',
                                      horizontalalignment='center')

        self.axes_height.set_ylabel("height (%s %s)" % (self.units_label(), self.reference))
        self.axes_height.grid(True)
        self.axes_height.legend(loc='best', fontsize=8)

        self.axes_agl.plot(xs, agl_plot, color='g', linewidth=1.5)
        self.axes_agl.axhline(0, color='r', linewidth=1)
        self.axes_agl.set_ylabel("height above ground (%s)" % self.units_label())
        self.axes_agl.set_xlabel("distance along mission (km)")
        self.axes_agl.grid(True)

        # keep the samples so the cursor readout can look values up
        self.plot_xs = xs
        self.plot_vehicle = veh_plot
        self.plot_terrain = ter_plot
        self.plot_agl = agl_plot
        self.make_readouts()

        known_agl = [a for a in agl_plot if a == a]
        if self.terrain_incomplete:
            self.label_status.SetLabel("waiting on %s terrain..." % self.source)
        elif known_agl:
            status = "min height above ground %.0f%s" % (min(known_agl),
                                                         self.units_label())
            if self.profile_width > 0:
                status += " within %.0fm of track" % self.profile_width
            if self.subsampled:
                # never let a coarser sample grid pass for full coverage
                status += " (sampled every %.0fm)" % self.cross_spacing
            self.label_status.SetLabel(status)
        else:
            self.label_status.SetLabel("")

        self.canvas.draw()
