from picamera import PiCamera
from time import sleep
import ps_drone
import time
import signal
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
import numpy as np

def kill_function(signal,frame):
    print("drone is shut down")
    drone.shutdown()

signal.signal(signal.SIGQUIT, kill_function)


i2c = busio.I2C(board.SCL, board.SDA)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

end = False

def on_press(key):
    global end
    end = True
    return False

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0] == -1):   time.sleep(0.1)           # Waits until drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1]))   # Gives the battery-status
if drone.getBattery()[1] == "empty":   sys.exit()                  # Give it up if battery is empty

drone.takeoff()
take pictures
camera = PiCamera()
camera.resolution = (640, 480)
sleep(2)
camera.capture('./Pic1.jpg')
sleep(1)
camera.capture('./Pic2.jpg')
camera.close()

while not end:
    print("Accelerometer (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
    print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic)
    x = accel.acceleration[0]
    #print("x= %0.3f"%x)
    y = accel.acceleration[1]
    #print("y= %0.3f"%y)
    z = accel.acceleration[2]
    #print("z= %0.3f"%z)
    roll = math.atan2(x,z);
    print ("roll = %0.3f"%roll+" radians")
    pitch = math.asin(x/9.81)
    print ("pitch= %0.3f"%pitch+" radians")
    print(" ")
    sleep(3)

time.sleep(9)

drone.shutdown()