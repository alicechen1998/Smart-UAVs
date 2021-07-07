import time, sys
import ps_drone												# Import PS-Drone-API
import cv2														# Import OpenCV

drone = ps_drone.Drone()										# Start using drone					
drone.startup()													# Connects to drone and starts subprocesses

drone.reset()													# Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0]==-1):	time.sleep(0.1)				# Waits until drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1]))	# Gives a battery-status
drone.useDemoMode(True)											# Just give me 15 basic dataset per second (is default anyway)

##### Mainprogram begin #####
drone.setConfigAllID()				# Go to multiconfiguration-mode
drone.sdVideo()						# Choose lower resolution (hdVideo() for...well, guess it)
drone.frontCam()					# Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.0001)	# Wait until it is done (after resync is done)
drone.startVideo()					# Start video-function

##### And action !
IMC = 	 drone.VideoImageCount		# Number of encoded videoframes
stop =	 False
while not stop:
	while drone.VideoImageCount==IMC: time.sleep(0.01)	# Wait until the next video-frame
	IMC = drone.VideoImageCount
	#key = drone.getKey()
	#if key:		stop = True
	img  = drone.VideoImage					# Copy video-image
	pImg = cv2.resize(img,(1000,1000))		# Process video-image
	cv2.imshow('Drones video',pImg)			# Show processed video-image
	cv2.waitKey(1)
