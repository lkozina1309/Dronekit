from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

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
		
def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
	

def condition_yaw(degrees,relative):
	if relative:
		is_relative = 1 #yaw relative to direction of travel
	else:
		is_relative = 0 #yaw is an absolute angle

	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		degrees, #Param 1, yaw in degrees
		0,  #Param 2, yaw speed deg/s
		1, #Param 3, Direction -1 ccw, 1 cw
		is_relative, # Param 4, relative offset 1, absolute angle 0
		0, 0, 0) # Param 5-7 not used
	vehicle.send_mavlink(msg)
	vehicle.flush()

		
    
arm_and_takeoff(1)

condition_yaw(30,1) ##180 -->> 210
print("Yawing 30 degrees relative to current position")
time.sleep(7)

counter=0
while counter<20:
	send_local_ned_velocity(5,0,0)
	time.sleep(1)
	print("Moving NORTH relative to front of drone")
	counter=counter+1

time.sleep(2)

condition_yaw(90,1) ##180 -->> 210
print("Yawing 90 degrees relative to current position")
time.sleep(7)

counter=0
while counter<20:
	send_local_ned_velocity(5,0,0)
	time.sleep(1)
	print("Moving NORTH relative to front of drone")
	counter=counter+1

time.sleep(2)

condition_yaw(90,1) ##180 -->> 210
print("Yawing 90 degrees relative to current position")
time.sleep(7)

counter=0
while counter<20:
	send_local_ned_velocity(5,0,0)
	time.sleep(1)
	print("Moving NORTH relative to front of drone")
	counter=counter+1

time.sleep(2)

condition_yaw(90,1) ##180 -->> 210
print("Yawing 90 degrees relative to current position")
time.sleep(7)

counter=0
while counter<20:
	send_local_ned_velocity(5,0,0)
	time.sleep(1)
	print("Moving NORTH relative to front of drone")
	counter=counter+1

time.sleep(2)

vehicle.mode    = VehicleMode("RTL")


