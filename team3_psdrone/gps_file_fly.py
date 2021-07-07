'''
GPS Interfacing with Raspberry Pi using Pyhton
http://www.electronicwings.com
'''

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
signal.signal(signal.SIGQUIT, kill_function)

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



def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    print("NMEA Time: ", nmea_time,'\n')
    print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.8f" %(position)
    return position

gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

try:
    while True:
        received_data = (str)(ser.readline())                   #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
            NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
            GPS_Info()                                          #get time, latitude, longitude
 
            print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
            map_link = 'http://maps.google.com/?q=' + lat_in_degrees + ',' + long_in_degrees    #create link to plot location on Google map
            print("<<<<<<<<press ctrl+c to plot location on google maps>>>>>>\n")               #press ctrl+c to plot on map and exit 
            print("------------------------------------------------------------\n")
                        
except KeyboardInterrupt:
    webbrowser.open(map_link)        #open current position information in google map
    sys.exit(0)

def haversine_distance(point1, point2):
    lon1,lat1 = (radians(coord) for coord in point1)
    lon2,lat2 = (radians(coord) for coord in point2)
    dlat = (lat2 - lat1)
    dlon = (lon2 - lon1)
    a = ( sin(dlat *0.5)**2 + cos(lat1) * cos(lat2)* sin (dlon *0.5)**2 )
    
    return 7917.5 * asin(sqrt(a))

def angle_of_vector (a,b,c,d):
    change_ofX = b - a
    change_ofY = d - c
    theta = atan(change_ofY/change_ofX)
    #print("arctan of y of x:",theta)
    final_theta = theta * (180/pi)
    #print ("angle in degree:()",final_theta)
    return final_theta




def moveCommand():
    
    
    #drone.takeoff()
    #time.sleep(10)

    #drone.moveForward(0.5)
    NDC = drone.NavDataCount
    end = False
    with open('myfile.txt','w') as file:
        
        try:
            x = 0
            while x < 5:      
                while drone.NavDataCount == NDC:  time.sleep(0.001)
                NDC=drone.NavDataCount
                received_data = (str)(ser.readline())                   #read NMEA string received
                GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
                if (GPGGA_data_available>0):
                    GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
                    NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                    GPS_Info()                                          #get time, latitude, longitude
                    print("lat in degrees = :",lat_in_degrees," long in degree:", long_in_degrees, '\n')
                    drone_yaw = drone.NavData["demo"][2][2]
                    if drone_yaw < 0:
                        drone_yaw +=360
                    map_link = 'http://maps.google.com/?q=' + lat_in_degrees + ',' + long_in_degrees    #create link to plot location on Google map
                    print("<<<<<<<<press ctrl+c to plot location on google maps>>>>>>\n")               #press ctrl+c to plot on map and exi
                    file.write(lat_in_degrees)
                    file.write('\n')
                    file.write(long_in_degrees)
                    file.write('\n')
                    file.write(str(drone_yaw))
                    file.write('\n')
                    print("------------------------------------------------------------\n")
                    x += 1
                    time.sleep(1)
            file.close()

        except KeyboardInterrupt:
            webbrowser.open(map_link)        #open current position information in google map
            sys.exit(0)
        
    #drone.stop()
    #time.sleep(2)
    
    
    #drone.turnAngle(95,0.3)         #turn angle with 95 degree and with speed 0.3
    with open('myfile.txt','a') as file:
        try:
            x = 0
            while x < 3:
                while drone.NavDataCount == NDC:  time.sleep(0.001)
                NDC=drone.NavDataCount
                received_data = (str)(ser.readline())                   #read NMEA string received
                GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
                if (GPGGA_data_available>0):
                    GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
                    NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                    GPS_Info()                                          #get time, latitude, longitude
                    print("lat in degrees = :",lat_in_degrees," long in degree:", long_in_degrees, '\n')
                    drone_yaw = drone.NavData["demo"][2][2]
                    if drone_yaw < 0:
                        drone_yaw +=360
                    print("current Heading angle:", drone_yaw)
                    map_link = 'http://maps.google.com/?q=' + lat_in_degrees + ',' + long_in_degrees    #create link to plot location on Google map
                    print("<<<<<<<<press ctrl+c to plot location on google maps>>>>>>\n")               #press ctrl+c to plot on map and exi
                    file.write(lat_in_degrees)
                    file.write('\n')
                    file.write(long_in_degrees)
                    file.write('\n')
                    file.write(str(drone_yaw))
                    file.write('\n')
                    print("------------------------------------------------------------\n")
                    x += 1
                    time.sleep(1)
            file.close()

        except KeyboardInterrupt:
            webbrowser.open(map_link)        #open current position information in google map
            sys.exit(0)
    #drone.sleep(5)
    #drone.stop()
    #drone.sleep(2)
    
    #drone.moveForward(0.1)
    with open('myfile.txt','a') as file:
        try:
            x = 0
            while x < 3:
                while drone.NavDataCount == NDC:  time.sleep(0.001)
                NDC=drone.NavDataCount
                received_data = (str)(ser.readline())                   #read NMEA string received
                GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
                if (GPGGA_data_available>0):
                    GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
                    NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                    GPS_Info()                                          #get time, latitude, longitude
                    print("lat in degrees = :",lat_in_degrees," long in degree:", long_in_degrees, '\n')
                    drone_yaw = drone.NavData["demo"][2][2]
                    if drone_yaw < 0:
                        drone_yaw +=360
                    map_link = 'http://maps.google.com/?q=' + lat_in_degrees + ',' + long_in_degrees    #create link to plot location on Google map
                    print("<<<<<<<<press ctrl+c to plot location on google maps>>>>>>\n")               #press ctrl+c to plot on map and exi
                    file.write(lat_in_degrees)
                    file.write('\n')
                    file.write(long_in_degrees)
                    file.write('\n')
                    file.write(str(drone_yaw))
                    file.write('\n')
                    print("------------------------------------------------------------\n")
                    x += 1
                    time.sleep(1)
            file.close()

        except KeyboardInterrupt:
            webbrowser.open(map_link)        #open current position information in google map
            sys.exit(0)
            
    #drone.stop()
    #time.sleep(3)



moveCommand()

with open('myfile.txt','r') as file:
    linesLine = file.readlines()
    first_latitude = linesLine[0]
    first_longitude= linesLine[1]
    last_latitude = linesLine[-3]
    last_longitude = linesLine[-2]
    drone_yaw_angle = linesLine[-1]
    
    file.close()

f_latitude = float(first_latitude)
f_longitude = float(first_longitude)
l_latitude = float(last_latitude)
l_longitude = float(last_longitude)
l_magnetic = float(drone_yaw_angle)

lisst = []
for i in range(0,1):
    lisst.append(f_latitude)
    lisst.append(f_longitude)

lisst1 = []
for i in range(0,1):
    lisst1.append(l_latitude)
    lisst1.append(l_longitude)

print ("distance",haversine_distance(lisst,lisst1))

angle_bwtween2angle = angle_of_vector(l_latitude,f_latitude,l_longitude,f_longitude)
 
print ("current yaw angle",drone_yaw)
print ("angle bewteen two coorindate:",angle_bwtween2angle)
#print ("angle from magnetic:",angle)
final_angle_tmp = (drone_yaw + angle_bwtween2angle)
print ("final angle to turn:", final_angle_tmp)


"""
if (final_angle_tmp > 180):
    final_angle = final_angle_tmp - 360
    print("after sub",final_angle)
    drone.turnAngle(final_angle,0.3)
    #drone.moveForward(0.3)
    #time.sleep(5)
    #drone.stop()
    #time.sleep(1)
else:
    drone.turnAngle(final_angle_tmp,0.3)
    drone.moveForward(0.3)
    time.sleep(5)
    drone.stop()
    time.sleep(1)
    

#drone.shutdown()
"""




