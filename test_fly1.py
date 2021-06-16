import time
import ps_drone                # Imports the PS-Drone-API
import cv2

print("Initializing")
drone = ps_drone.Drone()       # Initializes the PS-Drone-API
print("Starting")
drone.startup()                # Connects to the drone and starts subprocesses
print("Resetting")
drone.reset()
while drone.getBattery()[0] == -1:	time.sleep(0.1)		# Waits until the drone has done its reset
time.sleep(0.5)

print("Taking off")
drone.takeoff() # Drone starts
drone.turnLeft()
print("Sleeping")
time.sleep(7.5)                # Gives the drone time to start

print("Landing")
drone.land()                   # Drone lands
print("Done")
drone.shutdown()