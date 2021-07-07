import sys
#import geopy.bearing
from math import (
    degrees, radians, sin, cos, asin,
    tan, atan, atan2,pi, sqrt, exp,log,fabs
    )
from geo.constants import (
    EARTH_MEAN_RADIUS,
    EARTH_MEAN_DIAMETER)
print ("Input longitude and latitude")

def bearing(point1, point2):
    lon1,lat1 = (radians(coord) for coord in point1)
    lon2,lat2 = (radians(coord) for coord in point2)
    
    dlat = (lat2 - lat1)
    dlong = (lon2 - lon1)
    numerator = sin(dlong) *cos(lat2)
    denominator = (cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dlong))
                   )
    theta = atan2(numerator, denominator)
    theta_deg = (degrees(theta) + 360) % 360
    return theta_deg

def final_bearing(point1, point2):
    return (bearing(point2, point1) + 180) % 360


def py_approixmate_distance(point1, point2):
    "approixmate calculation"
    lon1,lat1 = (radians(coord) for coord in point1)
    lon2,lat2 = (radians(coord) for coord in point2)
    cos_lat = cos((lat1+lat2)/2.0)
    dx = (lat2 - lat1)
    dy = (cos_lat*(lon2-lon1))
    return 3958.8*sqrt(dx**2 + dy**2)

print (py_approixmate_distance([-73.93424,40.730610],[-85.480782,32.609856]))

def haversine_distance(point1, point2):
    lon1,lat1 = (radians(coord) for coord in point1)
    lon2,lat2 = (radians(coord) for coord in point2)
    dlat = (lat2 - lat1)
    dlon = (lon2 - lon1)
    a = ( sin(dlat *0.5)**2 + cos(lat1) * cos(lat2)* sin (dlon *0.5)**2 )
    
    return 7917.5 * asin(sqrt(a))

print (haversine_distance([-73.93424,40.730610],[-85.480782,32.609856]))
#output uisng  earth mean reaum 1367437.2111386023

lisst = []
for i in range(0,2):
    ele = float(input())
    
    lisst.append(ele)

lisst1 = []
for i in range(0,2):
    ele1 = float(input())
    
    lisst1.append(ele1)

#print(bearing(lisst,lisst1))
#print(final_bearing(lisst,lisst1))


