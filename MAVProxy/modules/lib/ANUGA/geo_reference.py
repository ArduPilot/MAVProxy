"""
This module adapted ANUGA
https://anuga.anu.edu.au/

"""


#FIXME: Ensure that all attributes of a georef are treated everywhere
#and unit test

import types, sys
import copy
import numpy as num


DEFAULT_ZONE = -1
TITLE = '#geo reference' + "\n" # this title is referred to in the test format

DEFAULT_PROJECTION = 'UTM'
DEFAULT_DATUM = 'wgs84'
DEFAULT_UNITS = 'm'
DEFAULT_FALSE_EASTING = 500000
DEFAULT_FALSE_NORTHING = 10000000    # Default for southern hemisphere


##
# @brief A class for ...
class Geo_reference:
    """
    Attributes of the Geo_reference class:
        .zone           The UTM zone (default is -1)
        .false_easting  ??
        .false_northing ??
        .datum          The Datum used (default is wgs84)
        .projection     The projection used (default is 'UTM')
        .units          The units of measure used (default metres)
        .xllcorner      The X coord of origin (default is 0.0 wrt UTM grid)
        .yllcorner      The y coord of origin (default is 0.0 wrt UTM grid)
        .is_absolute    ??

    """

    ##
    # @brief Instantiate an instance of class Geo_reference.
    # @param zone The UTM zone.
    # @param xllcorner X coord of origin of georef.
    # @param yllcorner Y coord of origin of georef.
    # @param datum ??
    # @param projection The projection used (default UTM).
    # @param units Units used in measuring distance (default m).
    # @param false_easting ??
    # @param false_northing ??
    # @param NetCDFObject NetCDF file *handle* to write to.
    # @param ASCIIFile ASCII text file *handle* to write to.
    # @param read_title Title of the georeference text.
    def __init__(self,
                 zone=DEFAULT_ZONE,
                 xllcorner=0.0,
                 yllcorner=0.0,
                 datum=DEFAULT_DATUM,
                 projection=DEFAULT_PROJECTION,
                 units=DEFAULT_UNITS,
                 false_easting=DEFAULT_FALSE_EASTING,
                 false_northing=DEFAULT_FALSE_NORTHING,
                 NetCDFObject=None,
                 ASCIIFile=None,
                 read_title=None):
        """
        input:
        NetCDFObject - a handle to the netCDF file to be written to
        ASCIIFile - a handle to the text file
        read_title - the title of the georeference text, if it was read in.
         If the function that calls this has already read the title line,
         it can't unread it, so this info has to be passed.
         If you know of a way to unread this info, then tell us.

         Note, the text file only saves a sub set of the info the
         points file does.  Currently the info not written in text
         must be the default info, since ANUGA assumes it isn't
         changing.
        """

        if zone is None:
            zone = DEFAULT_ZONE
        self.false_easting = int(false_easting)
        self.false_northing = int(false_northing)
        self.datum = datum
        self.projection = projection
        self.zone = int(zone)
        self.units = units
        self.xllcorner = float(xllcorner)
        self.yllcorner = float(yllcorner)

        if NetCDFObject is not None:
            self.read_NetCDF(NetCDFObject)

        if ASCIIFile is not None:
            self.read_ASCII(ASCIIFile, read_title=read_title)

        # Set flag for absolute points (used by get_absolute)
        self.absolute = num.allclose([self.xllcorner, self.yllcorner], 0)


    def get_xllcorner(self):
        return self.xllcorner

    ##
    # @brief Get the Y coordinate of the origin of this georef.
    def get_yllcorner(self):
        return self.yllcorner

    ##
    # @brief Get the zone of this georef.
    def get_zone(self):
        return self.zone

    ##
    # @brief Write <something> to an open NetCDF file.
    # @param outfile Handle to open NetCDF file.
    def write_NetCDF(self, outfile):
        outfile.xllcorner = self.xllcorner
        outfile.yllcorner = self.yllcorner
        outfile.zone = self.zone

        outfile.false_easting = self.false_easting
        outfile.false_northing = self.false_northing

        outfile.datum = self.datum
        outfile.projection = self.projection
        outfile.units = self.units

    ##
    # @brief Read data from an open NetCDF file.
    # @param infile Handle to open NetCDF file.
    def read_NetCDF(self, infile):
        self.xllcorner = float(infile.xllcorner[0])
        self.yllcorner = float(infile.yllcorner[0])
        self.zone = int(infile.zone[0])

        try:
            self.false_easting = int(infile.false_easting[0])
            self.false_northing = int(infile.false_northing[0])

            self.datum = infile.datum
            self.projection = infile.projection
            self.units = infile.units
        except:
            pass

        if self.false_easting != DEFAULT_FALSE_EASTING:
            print("WARNING: False easting of %f specified." % self.false_easting)
            print("Default false easting is %f." % DEFAULT_FALSE_EASTING)
            print("ANUGA does not correct for differences in False Eastings.")

        if self.false_northing != DEFAULT_FALSE_NORTHING:
            print("WARNING: False northing of %f specified."
                  % self.false_northing)
            print("Default false northing is %f." % DEFAULT_FALSE_NORTHING)
            print("ANUGA does not correct for differences in False Northings.")

        if self.datum.upper() != DEFAULT_DATUM.upper():
            print("WARNING: Datum of %s specified." % self.datum)
            print("Default Datum is %s." % DEFAULT_DATUM)
            print("ANUGA does not correct for differences in datums.")

        if self.projection.upper() != DEFAULT_PROJECTION.upper():
            print("WARNING: Projection of %s specified." % self.projection)
            print("Default Projection is %s." % DEFAULT_PROJECTION)
            print("ANUGA does not correct for differences in Projection.")

        if self.units.upper() != DEFAULT_UNITS.upper():
            print("WARNING: Units of %s specified." % self.units)
            print("Default units is %s." % DEFAULT_UNITS)
            print("ANUGA does not correct for differences in units.")

################################################################################
# ASCII files with geo-refs are currently not used
################################################################################

    ##
    # @brief Write georef data to an open text file.
    # @param fd Handle to open text file.
    def write_ASCII(self, fd):
        fd.write(TITLE)
        fd.write(str(self.zone) + "\n")
        fd.write(str(self.xllcorner) + "\n")
        fd.write(str(self.yllcorner) + "\n")

    ##
    # @brief Read georef data from an open text file.
    # @param fd Handle to open text file.
    def read_ASCII(self, fd, read_title=None):
        try:
            if read_title is None:
                read_title = fd.readline()     # remove the title line
            if read_title[0:2].upper() != TITLE[0:2].upper():
                msg = ('File error.  Expecting line: %s.  Got this line: %s'
                       % (TITLE, read_title))
                raise ValueError(msg)
            self.zone = int(fd.readline())
            self.xllcorner = float(fd.readline())
            self.yllcorner = float(fd.readline())
        except SyntaxError:
            msg = 'File error.  Got syntax error while parsing geo reference'
            raise ValueError(msg)

        # Fix some assertion failures
        if isinstance(self.zone, num.ndarray) and self.zone.shape == ():
            self.zone = self.zone[0]
        if (isinstance(self.xllcorner, num.ndarray) and
                self.xllcorner.shape == ()):
            self.xllcorner = self.xllcorner[0]
        if (isinstance(self.yllcorner, num.ndarray) and
                self.yllcorner.shape == ()):
            self.yllcorner = self.yllcorner[0]

        assert (type(self.xllcorner) == types.FloatType)
        assert (type(self.yllcorner) == types.FloatType)
        assert (type(self.zone) == types.IntType)

################################################################################

    ##
    # @brief Change points to be absolute wrt new georef 'points_geo_ref'.
    # @param points The points to change.
    # @param points_geo_ref The new georef to make points absolute wrt.
    # @return The changed points.
    # @note If 'points' is a list then a changed list is returned.
    def change_points_geo_ref(self, points, points_geo_ref=None):
        """Change the geo reference of a list or numeric array of points to
        be this reference.(The reference used for this object)
        If the points do not have a geo ref, assume 'absolute' values
        """
        import copy

        # remember if we got a list
        is_list = isinstance(points, list)

        points = ensure_numeric(points, num.float)

        # sanity checks
        if len(points.shape) == 1:
            #One point has been passed
            msg = 'Single point must have two elements'
            assert len(points) == 2, msg
            points = num.reshape(points, (1,2))

        msg = 'Points array must be two dimensional.\n'
        msg += 'I got %d dimensions' %len(points.shape)
        assert len(points.shape) == 2, msg

        msg = 'Input must be an N x 2 array or list of (x,y) values. '
        msg += 'I got an %d x %d array' %points.shape
        assert points.shape[1] == 2, msg

        # FIXME (Ole): Could also check if zone, xllcorner, yllcorner
        # are identical in the two geo refs.
        if points_geo_ref is not self:
            # If georeferences are different
            points = copy.copy(points) # Don't destroy input
            if not points_geo_ref is None:
                # Convert points to absolute coordinates
                points[:,0] += points_geo_ref.xllcorner
                points[:,1] += points_geo_ref.yllcorner

            # Make points relative to primary geo reference
            points[:,0] -= self.xllcorner
            points[:,1] -= self.yllcorner

        if is_list:
            points = points.tolist()

        return points

    def is_absolute(self):
        """Return True if xllcorner==yllcorner==0 indicating that points
        in question are absolute.
        """

        # FIXME(Ole): It is unfortunate that decision about whether points
        # are absolute or not lies with the georeference object. Ross pointed this out.
        # Moreover, this little function is responsible for a large fraction of the time
        # using in data fitting (something in like 40 - 50%.
        # This was due to the repeated calls to allclose.
        # With the flag method fitting is much faster (18 Mar 2009).

        # FIXME(Ole): HACK to be able to reuse data already cached (18 Mar 2009).
        # Remove at some point
        if not hasattr(self, 'absolute'):
            self.absolute = num.allclose([self.xllcorner, self.yllcorner], 0)

        # Return absolute flag
        return self.absolute

    def get_absolute(self, points):
        """Given a set of points geo referenced to this instance,
        return the points as absolute values.
        """

        # remember if we got a list
        is_list = isinstance(points, list)

        points = ensure_numeric(points, num.float)
        if len(points.shape) == 1:
            # One point has been passed
            msg = 'Single point must have two elements'
            if not len(points) == 2:
                raise ValueError(msg)


        msg = 'Input must be an N x 2 array or list of (x,y) values. '
        msg += 'I got an %d x %d array' %points.shape
        if not points.shape[1] == 2:
            raise ValueError(msg)


        # Add geo ref to points
        if not self.is_absolute():
            points = copy.copy(points) # Don't destroy input
            points[:,0] += self.xllcorner
            points[:,1] += self.yllcorner


        if is_list:
            points = points.tolist()

        return points

    ##
    # @brief Convert points to relative measurement.
    # @param points Points to convert to relative measurements.
    # @return A set of points relative to the geo_reference instance.
    def get_relative(self, points):
        """Given a set of points in absolute UTM coordinates,
        make them relative to this geo_reference instance,
        return the points as relative values.

        This is the inverse of get_absolute.
        """

        # remember if we got a list
        is_list = isinstance(points, list)

        points = ensure_numeric(points, num.float)
        if len(points.shape) == 1:
            #One point has been passed
            msg = 'Single point must have two elements'
            if not len(points) == 2:
                raise ValueError(msg)

        if not points.shape[1] == 2:
            msg = ('Input must be an N x 2 array or list of (x,y) values. '
                   'I got an %d x %d array' % points.shape)
            raise ValueError(msg)

        # Subtract geo ref from points
        if not self.is_absolute():
            points = copy.copy(points) # Don't destroy input
            points[:,0] -= self.xllcorner
            points[:,1] -= self.yllcorner

        if is_list:
            points = points.tolist()

        return points

    ##
    # @brief ??
    # @param other ??
    def reconcile_zones(self, other):
        if other is None:
            other = Geo_reference()
        if (self.zone == other.zone or
            self.zone == DEFAULT_ZONE and
            other.zone == DEFAULT_ZONE):
            pass
        elif self.zone == DEFAULT_ZONE:
            self.zone = other.zone
        elif other.zone == DEFAULT_ZONE:
            other.zone = self.zone
        else:
            msg = ('Geospatial data must be in the same '
                   'ZONE to allow reconciliation. I got zone %d and %d'
                   % (self.zone, other.zone))
            raise ValueError(msg)

    #def easting_northing2geo_reffed_point(self, x, y):
    #    return [x-self.xllcorner, y - self.xllcorner]

    #def easting_northing2geo_reffed_points(self, x, y):
    #    return [x-self.xllcorner, y - self.xllcorner]

    ##
    # @brief Get origin of this geo_reference.
    # @return (zone, xllcorner, yllcorner).
    def get_origin(self):
        return (self.zone, self.xllcorner, self.yllcorner)

    ##
    # @brief Get a string representation of this geo_reference instance.
    def __repr__(self):
        return ('(zone=%i easting=%f, northing=%f)'
                % (self.zone, self.xllcorner, self.yllcorner))

    ##
    # @brief Compare two geo_reference instances.
    # @param self This geo_reference instance.
    # @param other Another geo_reference instance to compare against.
    # @return 0 if instances have the same attributes, else 1.
    # @note Attributes are: zone, xllcorner, yllcorner.
    def __cmp__(self, other):
        # FIXME (DSG) add a tolerance
        if other is None:
            return 1
        cmp = 0
        if not (self.xllcorner == self.xllcorner):
            cmp = 1
        if not (self.yllcorner == self.yllcorner):
            cmp = 1
        if not (self.zone == self.zone):
            cmp = 1
        return cmp


##
# @brief Write a geo_reference to a NetCDF file (usually SWW).
# @param origin A georef instance or parameters to create a georef instance.
# @param outfile Path to file to write.
# @return A normalized geo_reference.
def write_NetCDF_georeference(origin, outfile):
    """Write georeference info to a netcdf file, usually sww.

    The origin can be a georef instance or parameters for a geo_ref instance

    outfile is the name of the file to be written to.
    """

    geo_ref = ensure_geo_reference(origin)
    geo_ref.write_NetCDF(outfile)
    return geo_ref


##
# @brief Convert an object to a georeference instance.
# @param origin A georef instance or (zone, xllcorner, yllcorner)
# @return A georef object, or None if 'origin' was None.
def ensure_geo_reference(origin):
    """
    Given a list/tuple of zone, xllcorner and yllcorner of a geo-ref object,
    return a geo ref object.

    If the origin is None, return None, so calling this function doesn't
    effect code logic
    """

    if isinstance(origin, Geo_reference):
        geo_ref = origin
    elif origin is None:
        geo_ref = None
    else:
        geo_ref = apply(Geo_reference, origin)

    return geo_ref


#-----------------------------------------------------------------------

if __name__ == "__main__":
    pass
