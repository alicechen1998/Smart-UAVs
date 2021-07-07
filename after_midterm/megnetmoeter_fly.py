import time, sys
import ps_drone      # Import PS-Drone-API
import signal


def kill_function(signal,frame):
    print("drone is shut down")
    drone.shutdown()

signal.signal(signal.SIGQUIT, kill_function)

drone = ps_drone.Drone()                                         # Start using drone					
drone.startup()                                                  # Connects to drone and starts subprocesses

drone.reset()                                                    # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):   time.sleep(0.1)           # Waits until drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1]))   # Gives the battery-status
if drone.getBattery()[1] == "empty":   sys.exit()                # Give it up if battery is empty
drone.useDemoMode(False)
drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi","wind_speed","euler_angles"])       # Packets, which shall be decoded
time.sleep(1.0)                                                               # Give it some time to awake fully after reset

##### Mainprogram begin #####
drone.takeoff()
time.sleep(3)
NDC = drone.NavDataCount
end = False
while not end:
    while drone.NavDataCount == NDC:  time.sleep(0.001)                       # Wait until next time-unit
    #if drone.getKey():                end = True                              # Stop if any key is pressed
    NDC=drone.NavDataCount
    print ("-----------")
    print ("Aptitude [X,Y,Z] :            "+str(drone.NavData["demo"][2]))
    #print ("Altitude / sensor / pressure: "+str(drone.NavData["altitude"][3])+" / "+str(drone.State[21])+" / "+str(drone.NavData["pressure_raw"][0]))
    #print ("Megnetometer [X,Y,Z]:         "+str(drone.NavData["magneto"][0]))
    #print ("Wifi link quality:            "+str(drone.NavData["wifi"]))
    print ("wind speed:                   "+str(drone.NavData["wind_speed"][1]))
    print ("euler_angle:                  "+str(drone.NavData["euler_angle"][3]))
    
    
drone.shutdown()


