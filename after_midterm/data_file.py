import os
import time
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
import numpy as np
import time, sys
import ps_drone                                                              # Import PS-Drone-API
from time import sleep
from math import (
    degrees, radians, sin, cos, asin,
    tan, atan, atan2,pi, sqrt, exp,log,fabs
    )

drone = ps_drone.Drone()                                                      # Start using drone					
drone.startup() 

i2c = busio.I2C(board.SCL, board.SDA)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

home_dir = os.path.expanduser('~')
repository_dir = os.path.join(home_dir, 'Desktop/team3_psdrone')


end = False

def shutdown_gracefully(signal, frame):
    exit()

signal.signal(signal.SIGQUIT, shutdown_gracefully)

with open(os.path.join(repository_dir, 'temp.txt'),'w') as file:
    file.write("")
file.close()
f_longitude = 32.234211
f_latitude = 85.836133
l_longitude = 36.134134
l_latitude = 85.58723421

def angle_of_vector (a,b,c,d):
    dotProduct = a*c + b*d
    modOfVector1 = math.sqrt (a*a + b*b)*math.sqrt(c*c + d*d)
    angle = dotProduct/modOfVector1
  #  print("radian = ", angle)
    angleInDegree = math.degrees(math.acos(angle))
    print("Degree =:",angleInDegree)
    return angleInDegree

two_angle = angle_of_vector(f_longitude,f_latitude,l_longitude,l_latitude)
#print (two_angle)
def final_angls(a,b):
     c = a + b
     return c

with open(os.path.join(repository_dir, 'temp.txt'),'a') as file:
    while not end:
        #print("Accelerometer (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%(accel.acceleration[0],accel.acceleration[1],accel.acceleration[2]))
        print("Magnetometer (micro-teslas): X=%4.1f Y=%4.1f Z=%4.1f"%(mag.magnetic[0],mag.magnetic[1],mag.magnetic[2]))
        #roll = math.atan2(accel.acceleration[0],accel.acceleration[2])
        #pitch = math.asin(accel.acceleration[0]/9.81)
        angle = math.degrees(math.atan2(mag.magnetic[1],mag.magnetic[0]))
        angle+=137
        if angle<0:
            angle+=360
       # acceleration = (math.sqrt(accel.acceleration[0]**2 + accel.acceleration[1]**2 + accel.acceleration[2]**2) - 9.81)
        #print("Angle (degrees) from the magnetic: "+str(angle))
        #print("Acceleration: " + str(acceleration))
        #file.write(str(roll)+",")
        #file.write('\n')
        #file.write(str(angle)+",")
        #file.write('\n')
        #file.write(str(pitch)+"\n")
        time.sleep(0.5)
        #finalangle = final_angls(angle,two_angle)
        #print ("final turn angle ",finalangle)
file.close()

with open('temp.txt','r') as file:
    linesLine = file.readlines()
    last_magnetic = linesLine[-1]
    file.close()
print ("finally magnectic angle:",last_magnetic)

"""
def compass():
    #roll = math.atan2(accel.acceleration[0],accel.acceleration[2])
    #pitch = math.asin(accel.acceleration[0]/9.81)
    print("Magnetometer (micro-teslas): X=%4.1f Y=%4.1f Z=%4.1f"%(mag.magnetic[0],mag.magnetic[1],mag.magnetic[2]))
    
    

while not end:
    compass()
    time.sleep(1)

"""
