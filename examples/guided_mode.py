# Script guided_mode.py serves primarily as a template. 
# It is useful when I want to make script in GUIDED mode. 
# It contains all the functions that can be used in GUIDED mode, so anyone can choose and combine functions wanted.
# It also uses python-picamera to record a video to a file.
 



from __future__ import print_function
import time
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
		

def yaw(heading, relative=False):
	if relative:
		is_relative = 1
	else:
		is_relative = 0 
		
	msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0, 
        heading,    
        0,          
        1,          
        is_relative, 
        0, 0, 0)    
    
	vehicle.send_mavlink(msg)
    	
def velocity(velocity_x, velocity_y, velocity_z, duration):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)

	for x in range(0,duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)
		
def global_velocity(velocity_x, velocity_y, velocity_z, duration):
	msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,   
        0, 0,    
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
        0b0000111111000111,
        0, 
        0, 
        0, 
        velocity_x, 
        velocity_y, 
        velocity_z,
        0, 0, 0, 
        0, 0) 

	for x in range(0,duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)
		




arm_and_takeoff(10)

vehicle.airspeed=8
vehicle.simple_goto(LocationGlobalRelative(45.49357051270179, 18.093085661413, 20))
time.sleep(50)

global_velocity(-5,0,0,50)

yaw(90,relative=True)


velocity(0,10,0,50)

vehicle.mode    = VehicleMode("RTL")

