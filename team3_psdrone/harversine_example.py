import os
import serial               #import serial pacakge
from time import sleep
import webbrowser           #import package for opening link in browser
import sys
import time #import system package
import signal
import math
import time
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import numpy as np
import time, sys
import ps_drone                                                              # Import PS-Drone-API                                                   # Import PS-Drone-API
import signal
#import cv2
from math import (
    degrees, radians, sin, cos, asin,
    tan, atan, atan2,pi, sqrt, exp,log,fabs
    )

def haversine_distance(point1, point2):
    lon1,lat1 = (radians(coord) for coord in point1)
    lon2,lat2 = (radians(coord) for coord in point2)
    dlat = (lat2 - lat1)
    dlon = (lon2 - lon1)
    a = ( sin(dlat *0.5)**2 + cos(lat1) * cos(lat2)* sin (dlon *0.5)**2 )
    
    return 6371000/1000 * asin(sqrt(a))

print ("distance",haversine_distance([-34.8333333,-58.5166646],[49.0083899664,2.53844117956]))
