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
#import cv2

drone = ps_drone.Drone()                                                      # Start using drone					
drone.startup()                                                               # Connects to drone and starts subprocesses

drone.reset()                                                                 # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):  time.sleep(0.1)                         # Wait until the drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])) # Gives a battery-status
drone.useDemoMode(False)                                                      # Give me everything...fast
drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi","wind_speed","euler_angles"])       # Packets, which shall be decoded
time.sleep(1.0)

home_dir = os.path.expanduser('~')
repository_dir = os.path.join(home_dir, 'Desktop/team3_psdrone')
#print ("current path")
#print(repository_dir)
#print()

def shutdown_gracefully(signal, frame):
    exit()

signal.signal(signal.SIGQUIT, shutdown_gracefully)

def kill_function(signal,frame):
    #print("drone is shut down")
    drone.shutdown()

signal.signal(signal.SIGQUIT, kill_function)

def angle_of_vector (a,b,c,d):
    change_ofX = b - a
    change_ofY = d - c
    theta = atan(change_ofY/change_ofX)
    #print("arctan of y of x:",theta)
    final_theta = theta * (180/pi)
    #print ("angle in degree:()",final_theta)
    return final_theta


NDC = drone.NavDataCount
end = False

drone.takeoff()
time.sleep(10)

with open('myfile.txt','r') as file:
    linesLine = file.readlines()
    first_latitude = linesLine[0]
    first_longitude= linesLine[1]
    first_yaw_angle = linesLine[2]
    last_latitude = linesLine[-3]
    last_longitude = linesLine[-2]
    drone_yaw_angle = linesLine[-1]
    
    file.close()

f_latitude = float(first_latitude)
f_longitude = float(first_longitude)
l_latitude = float(last_latitude)
l_longitude = float(last_longitude)
l_magnetic = float(drone_yaw_angle)
f_yawAngle = float(first_yaw_angle)

lisst = []
for i in range(0,1):
    lisst.append(f_latitude)
    lisst.append(f_longitude)

lisst1 = []
for i in range(0,1):
    lisst1.append(l_latitude)
    lisst1.append(l_longitude)

#print ("distance",haversine_distance(lisst,lisst1))

angle_bwtween2angle = angle_of_vector(l_latitude,f_latitude,l_longitude,f_longitude)

#print ("current yaw angle",drone_yaw)
print ("angle bewteen two coorindate:",angle_bwtween2angle)
#print ("angle from magnetic:",angle)
final_angle_tmp = (l_magnetic + angle_bwtween2angle)
print ("final angle to turn:", final_angle_tmp)


if (final_angle_tmp > 180):
    final_angle = final_angle_tmp - 360
    print("after sub",final_angle)
    drone.turnAngle(final_angle,0.3)
    drone.moveForward(1)
else:
    drone.turnAngle(final_angle_tmp,0.3)
    drone.moveForward(1)


drone.shutdown()






