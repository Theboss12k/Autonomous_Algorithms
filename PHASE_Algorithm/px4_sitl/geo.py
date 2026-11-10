"""Flat-earth ENU <-> lat/lon around a reference point (fine for < 1 km)."""
import math

EARTH_R = 6378137.0
# PX4's default SIH home (SIH_LOC_LAT0 / SIH_LOC_LON0).
REF_LAT = 47.397742
REF_LON = 8.545594


def enu_to_latlon(east, north, ref_lat=REF_LAT, ref_lon=REF_LON):
    lat = ref_lat + math.degrees(north / EARTH_R)
    lon = ref_lon + math.degrees(east / (EARTH_R * math.cos(math.radians(ref_lat))))
    return lat, lon


def latlon_to_enu(lat, lon, ref_lat=REF_LAT, ref_lon=REF_LON):
    north = math.radians(lat - ref_lat) * EARTH_R
    east = math.radians(lon - ref_lon) * EARTH_R * math.cos(math.radians(ref_lat))
    return east, north
