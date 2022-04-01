from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=57600)

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

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5
	
def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.05:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None

loc = [LocationGlobalRelative(45.501044, 18.099386, 10), LocationGlobalRelative(45.501206, 18.099816, 10), LocationGlobalRelative(45.500984, 18.099945, 10), LocationGlobalRelative(45.500890, 18.099586 10)]

		
while (vehicle.mode != VehicleMode("GUIDED")):
    print("Waiting for GUIDED mode") 
    time.sleep(2)
    
arm_and_takeoff(10)

for i in range(len(loc)):
	print("Going to next destination")
	goto(loc[i])

vehicle.mode    = VehicleMode("RTL")

