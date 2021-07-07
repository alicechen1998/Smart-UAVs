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

def shutdown_gracefully(signal, frame):
    exit()

signal.signal(signal.SIGQUIT, shutdown_gracefully)

def kill_function(signal,frame):
    #print("drone is shut down")
    drone.shutdown()

drone = ps_drone.Drone()                                                      # Start using drone					
drone.startup()                                                               # Connects to drone and starts subprocesses

drone.reset()                                                                 # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):  time.sleep(0.1)                         # Wait until the drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])) # Gives a battery-status
drone.useDemoMode(False)                                                      # Give me everything...fast
drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi","wind_speed","euler_angles"])       # Packets, which shall be decoded
time.sleep(1.0)                          # Give it some time to awake fully after reset

#i2c = busio.I2C(board.SCL, board.SDA)
#mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
#accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
NDC = drone.NavDataCount
end = False

home_dir = os.path.expanduser('~')
repository_dir = os.path.join(home_dir, 'Desktop/team3_psdrone')
#print ("current path")
#print(repository_dir)
#print()
with open(os.path.join(repository_dir, 'testfile.txt'),'w') as file:
    file.write("")
file.close()
def moveCommand():
    drone.takeoff()
    time.sleep(5)

    drone.moveForward(0.5)
    with open('testfile.txt','w') as file:
        
        try:
            x = 0
            while x < 5:
                #while drone.NavDataCount == NDC:  time.sleep(0.001)
                NDC=drone.NavDataCount
                #received_data = (str)(ser.readline())                   #read NMEA string received             
                drone_yaw = drone.NavData["demo"][2][2]
                if drone_yaw < 0:
                    drone_yaw +=360
                    file.write(str(drone_yaw))
                    file.write('\n')
                    print("------------------------------------------------------------\n")
                    x += 1
                    time.sleep(1)
                
            file.close()
        except KeyboardInterrupt:
                print("error")
                sys.exit(0)
    drone.stop()
    time.sleep(2)
    
moveCommand()
drone.shutdown()