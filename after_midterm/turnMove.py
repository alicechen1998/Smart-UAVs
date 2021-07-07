import time, sys
import ps_drone                                                  # Import PS-Drone-API

drone = ps_drone.Drone()                                         # Start using drone					
drone.startup()                                                  # Connects to drone and starts subprocesses

drone.reset()                                                    # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):   time.sleep(0.1)           # Waits until drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1]))   # Gives the battery-status
if drone.getBattery()[1] == "empty":   sys.exit()                # Give it up if battery is empty
 
drone.useDemoMode(True)                                          # Just give me 15 basic dataset per second (is default anyway)
drone.getNDpackage(["demo"])                                     # Packets, which shall be decoded
time.sleep(0.5)                                                  # Give it some time to awake fully after reset

drone.trim()                                                     # Recalibrate sensors
drone.getSelfRotation(5)                                         # Getting value for auto-alteration of gyroscope-sensor
print ("Auto-alternation: "+str(drone.selfRotation)+" dec/sec" )   # Showing value for auto-alteration

drone.takeoff()                                                  # Fly, drone, fly !
while drone.NavData["demo"][0][2]:     time.sleep(0.1)           # Wait until the drone is really flying (not in landed-mode anymore)

##### Mainprogram begin #####
print ("Drone is flying now")

print ("Drone does a complex movement")
leftRight       = -0.02 # Move with 2% speed to the left
#backwardForward = -0.1                                           # Move with 10% speed backwards
#downUp          = 0.3                                            # Move with 30% speed upward
#turnLeftRight   = 1                                              # Turn full speed right
#drone.move(leftRight)#, #backwardForward, downUp, turnLeftRight)    # Do movement
drone.shutdown()
#timeToFlight    = 2.5                                            # Time to fly at all 
#refTime         = time.time()                                    # Start-time
#end             = False
while not end:
    if drone.getKey():                        sys.exit()         # Stop when any key is pressed
    if time.time()-refTime >= timeToFlight:   end = True         # Stop when max time of flight has been reached

#print ("Drone stopped movement")
#drone.stop()
#time.sleep(2)

#print ("Drone turns 120 degree to the left")
#drone.turnAngle(-50,1,1)

#drone.shutdown()
