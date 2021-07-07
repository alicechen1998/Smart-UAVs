import time, sys
import ps_drone                                                              # Import PS-Drone-API
from time import sleep
from math import (
    degrees, radians, sin, cos, asin,
    tan, atan, atan2,pi, sqrt, exp,log,fabs
    )
drone = ps_drone.Drone()                                                      # Start using drone					
drone.startup()                                                               # Connects to drone and starts subprocesses

drone.reset()                                                                 # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):  time.sleep(0.1)                         # Wait until the drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])) # Gives a battery-status
drone.useDemoMode(False)                                                      # Give me everything...fast
drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi","wind_speed","euler_angles"])       # Packets, which shall be decoded
time.sleep(1.0)                                                               # Give it some time to awake fully after reset

##### Mainprogram begin #####
NDC = drone.NavDataCount
end = False
while not end:
    while drone.NavDataCount == NDC:  time.sleep(0.001)                       # Wait until next time-unit
    #if drone.getKey():                end = True                              # Stop if any key is pressed
    NDC=drone.NavDataCount
    print ("-----------")
    print ("Aptitude [X,Y,Z] :            "+str(drone.NavData["demo"][2]))
    print ("Altitude / sensor / pressure: "+str(drone.NavData["altitude"][3])+" / "+str(drone.State[21])+" / "+str(drone.NavData["pressure_raw"][0]))
    print ("Megnetometer [X,Y,Z]:         "+str(drone.NavData["magneto"][0]))
    #print ("Megnetometer [X,Y]:           "+str(drone.NavData["magneto"][0][0]))
    #print ("Megnetometer [X,Y]:           "+str(drone.NavData["magneto"][0][1]))
    print ("Wifi link quality:            "+str(drone.NavData["wifi"]))
    print ("wind speed:                   "+str(drone.NavData["wind_speed"][1]))
    print ("euler_angles:                 "+str(drone.NavData["euler_angles"][1]))
    drone_yaw = drone.NavData["demo"][2][2]
    if drone_yaw < 0:
        drone_yaw +=360
    #mag_x = float(drone.NavData["magneto"][0][0])
    #mag_y = float(drone.NavData["magneto"][0][1])
    #mag_add= mag_x + mag_y
    #heading_angle = atan2(mag_y,mag_x)
    print ("current yaw angle",drone_yaw)
    sleep(1)