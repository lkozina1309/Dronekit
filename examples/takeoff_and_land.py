

# This is a script for simple takeoff to 10 meters and landing after connecting and showing state of the vehicle. 


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

vehicle = connect('/dev/ttyAMA0'', wait_ready=True, baud=57600)


print ("Global Location: %s" % vehicle.location.global_frame)
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print ("Local Location: %s" % vehicle.location.local_frame)    #
print ("Attitude: %s" % vehicle.attitude)
print ("Velocity: %s" % vehicle.velocity)
print ("GPS: %s" % vehicle.gps_0)
print ("Groundspeed: %s" % vehicle.groundspeed)
print ("Airspeed: %s" % vehicle.airspeed)
print ("Gimbal status: %s" % vehicle.gimbal)
print ("Battery: %s" % vehicle.battery)
print ("EKF OK?: %s" % vehicle.ekf_ok)
print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
print ("Rangefinder: %s" % vehicle.rangefinder)
print ("Rangefinder distance: %s" % vehicle.rangefinder.distance)
print ("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print ("Heading: %s" % vehicle.heading)
print ("Is Armable?: %s" % vehicle.is_armable)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name)    
print ("Armed: %s" % vehicle.armed)    

def arm_and_takeoff(altitude):
	while not vehicle.is_armable:
        	print (" Waiting for vehicle to initialise...")
        	time.sleep(1)
        
	print ("Arming motors")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	
	while not vehicle.armed:
        	print (" Waiting for arming...")
        	time.sleep(1)

	print ("Taking off!")
	vehicle.simple_takeoff(altitude)
	
	while True:
		print (" Altitude: ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=altitude*0.95:
			print ("Reached target altitude")
			break
		time.sleep(1) 
		  
arm_and_takeoff(10)
vehicle.mode    = VehicleMode("LAND")  
