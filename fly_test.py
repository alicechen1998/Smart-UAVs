import ps_drone
import time
import signal

def kill_function(signal,frame):
    print("drone is shut down")
    drone.shutdown()

signal.signal(signal.SIGQUIT, kill_function)


drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0] == -1):   time.sleep(0.1)           # Waits until drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1]))   # Gives the battery-status
if drone.getBattery()[1] == "empty":   sys.exit()                  # Give it up if battery is empty

drone.takeoff()
#drone.setSpeed(0.1)
#drone.moveLeft(0.1)
#drone.moveForward(0.1)
#print("drone is moving forward")
time.sleep(9)


print("drone is landing")
drone.land()
drone.shutdown()