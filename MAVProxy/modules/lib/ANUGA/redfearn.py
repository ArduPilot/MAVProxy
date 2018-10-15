"""
This module adapted ANUGA
https://anuga.anu.edu.au/

------------

Implementation of Redfearn's formula to compute UTM projections from latitude and longitude

Based in part on spreadsheet
www.icsm.gov.au/gda/gdatm/redfearn.xls
downloaded from INTERGOVERNMENTAL COMMITTEE ON SURVEYING & MAPPING (ICSM)
http://www.icsm.gov.au/icsm/

"""
from MAVProxy.modules.lib.ANUGA.geo_reference import Geo_reference, DEFAULT_ZONE


def degminsec2decimal_degrees(dd,mm,ss):
    assert abs(mm) == mm
    assert abs(ss) == ss

    if dd < 0:
        sign = -1
    else:
        sign = 1

    return sign * (abs(dd) + mm/60. + ss/3600.)

def decimal_degrees2degminsec(dec):

    if dec < 0:
        sign = -1
    else:
        sign = 1

    dec = abs(dec)
    dd = int(dec)
    f = dec-dd

    mm = int(f*60)
    ss = (f*60-mm)*60

    return sign*dd, mm, ss

def redfearn(lat, lon, false_easting=None, false_northing=None,
             zone=None, central_meridian=None, scale_factor=None):
    """Compute UTM projection using Redfearn's formula

    lat, lon is latitude and longitude in decimal degrees

    If false easting and northing are specified they will override
    the standard

    If zone is specified reproject lat and long to specified zone instead of
    standard zone

    If meridian is specified, reproject lat and lon to that instead of zone. In this case
    zone will be set to -1 to indicate non-UTM projection

    Note that zone and meridian cannot both be specifed
    """


    from math import pi, sqrt, sin, cos, tan



    #GDA Specifications
    a = 6378137.0                       #Semi major axis
    inverse_flattening = 298.257222101  #1/f
    if scale_factor is None:
        K0 = 0.9996                         #Central scale factor
    else:
        K0 = scale_factor
    #print('scale', K0)
    zone_width = 6                      #Degrees

    longitude_of_central_meridian_zone0 = -183
    longitude_of_western_edge_zone0 = -186

    if false_easting is None:
        false_easting = 500000

    if false_northing is None:
        if lat < 0:
            false_northing = 10000000  #Southern hemisphere
        else:
            false_northing = 0         #Northern hemisphere)


    #Derived constants
    f = 1.0/inverse_flattening
    b = a*(1-f)       #Semi minor axis

    e2 = 2*f - f*f#    = f*(2-f) = (a^2-b^2/a^2   #Eccentricity
    e = sqrt(e2)
    e2_ = e2/(1-e2)   # = (a^2-b^2)/b^2 #Second eccentricity
    e_ = sqrt(e2_)
    e4 = e2*e2
    e6 = e2*e4

    #Foot point latitude
    n = (a-b)/(a+b) #Same as e2 - why ?
    n2 = n*n
    n3 = n*n2
    n4 = n2*n2

    G = a*(1-n)*(1-n2)*(1+9*n2/4+225*n4/64)*pi/180


    phi = lat*pi/180     #Convert latitude to radians

    sinphi = sin(phi)
    sin2phi = sin(2*phi)
    sin4phi = sin(4*phi)
    sin6phi = sin(6*phi)

    cosphi = cos(phi)
    cosphi2 = cosphi*cosphi
    cosphi3 = cosphi*cosphi2
    cosphi4 = cosphi2*cosphi2
    cosphi5 = cosphi*cosphi4
    cosphi6 = cosphi2*cosphi4
    cosphi7 = cosphi*cosphi6
    cosphi8 = cosphi4*cosphi4

    t = tan(phi)
    t2 = t*t
    t4 = t2*t2
    t6 = t2*t4

    #Radius of Curvature
    rho = a*(1-e2)/(1-e2*sinphi*sinphi)**1.5
    nu = a/(1-e2*sinphi*sinphi)**0.5
    psi = nu/rho
    psi2 = psi*psi
    psi3 = psi*psi2
    psi4 = psi2*psi2



    #Meridian distance

    A0 = 1 - e2/4 - 3*e4/64 - 5*e6/256
    A2 = 3.0/8*(e2+e4/4+15*e6/128)
    A4 = 15.0/256*(e4+3*e6/4)
    A6 = 35*e6/3072

    term1 = a*A0*phi
    term2 = -a*A2*sin2phi
    term3 = a*A4*sin4phi
    term4 = -a*A6*sin6phi

    m = term1 + term2 + term3 + term4 #OK

    if zone is not None and central_meridian is not None:
        msg = 'You specified both zone and central_meridian. Provide only one of them'
        raise ValueError(msg)

    # Zone
    if zone is None:
        zone = int((lon - longitude_of_western_edge_zone0)/zone_width)

    # Central meridian
    if central_meridian is None:
        central_meridian = zone*zone_width+longitude_of_central_meridian_zone0
    else:
        zone = -1

    omega = (lon-central_meridian)*pi/180 #Relative longitude (radians)
    omega2 = omega*omega
    omega3 = omega*omega2
    omega4 = omega2*omega2
    omega5 = omega*omega4
    omega6 = omega3*omega3
    omega7 = omega*omega6
    omega8 = omega4*omega4

    #Northing
    term1 = nu*sinphi*cosphi*omega2/2
    term2 = nu*sinphi*cosphi3*(4*psi2+psi-t2)*omega4/24
    term3 = nu*sinphi*cosphi5*\
            (8*psi4*(11-24*t2)-28*psi3*(1-6*t2)+\
             psi2*(1-32*t2)-psi*2*t2+t4-t2)*omega6/720
    term4 = nu*sinphi*cosphi7*(1385-3111*t2+543*t4-t6)*omega8/40320
    northing = false_northing + K0*(m + term1 + term2 + term3 + term4)

    #Easting
    term1 = nu*omega*cosphi
    term2 = nu*cosphi3*(psi-t2)*omega3/6
    term3 = nu*cosphi5*(4*psi3*(1-6*t2)+psi2*(1+8*t2)-2*psi*t2+t4)*omega5/120
    term4 = nu*cosphi7*(61-479*t2+179*t4-t6)*omega7/5040
    easting = false_easting + K0*(term1 + term2 + term3 + term4)

    return zone, easting, northing



def convert_from_latlon_to_utm(points=None,
                               latitudes=None,
                               longitudes=None,
                               false_easting=None,
                               false_northing=None):
    """Convert latitude and longitude data to UTM as a list of coordinates.


    Input

    points: list of points given in decimal degrees (latitude, longitude) or
    latitudes: list of latitudes   and
    longitudes: list of longitudes
    false_easting (optional)
    false_northing (optional)

    Output

    points: List of converted points
    zone:   Common UTM zone for converted points


    Notes

    Assume the false_easting and false_northing are the same for each list.
    If points end up in different UTM zones, an ANUGAerror is thrown.
    """

    old_geo = Geo_reference()
    utm_points = []
    if points is None:
        assert len(latitudes) == len(longitudes)
        points =  map(None, latitudes, longitudes)

    for point in points:

        zone, easting, northing = redfearn(float(point[0]),
                                           float(point[1]),
                                           false_easting=false_easting,
                                           false_northing=false_northing)
        new_geo = Geo_reference(zone)
        old_geo.reconcile_zones(new_geo)
        utm_points.append([easting, northing])

    return utm_points, old_geo.get_zone()
