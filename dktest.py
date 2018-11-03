# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 00:55:12 2018

@author: drkfr
"""
# Import all the needed libraries
from __future__ import print_function
import time
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative

###############################################################################
# Take in any arguments and connect to the sitl / platform
##############################################################################
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


###############################################################################
# user defined functions
###############################################################################
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)
        

def get_vehicle_states():
    # vehicle is an instance of the Vehicle class
    print( "Autopilot Firmware version: %s" % vehicle.version)
    print ("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
    print ("Global Location: %s" % vehicle.location.global_frame)
    print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print ("Local Location: %s" % vehicle.location.local_frame)    #NED
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
    print ("Mode: %s" % vehicle.mode.name)    # settable
    print ("Armed: %s" % vehicle.armed )   # settable
    
def set_roi(location):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
	
	
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;
	
	
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
	

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;
	
#def ChangeMode(vehicle,mode):
#	while vehicle.mode != VehicleMode(mode):
#		vehicle.mode = VehicleMode(mode):
#		time.sleep(0.5)
	
###############################################################################
# MAIN Script
###############################################################################
print("Getting vehicle states")
get_vehicle_states()

print("5 seconds to arm")
time.sleep(5)

print("Arm and Takeoff")
arm_and_takeoff(5)

vehicle_airspeed = 5
print("Setting vehicle airspeed :%s" % vehicle_airspeed)
vehicle.airspeed = vehicle_airspeed

print("5 seconds to first waypoint")
time.sleep(5)

print("Going towards first point")
point1 = LocationGlobalRelative(37.376941, -120.411421, 20)
vehicle.simple_goto(point1)
time.sleep(20)

print("Going towards second point(groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(37.378477, -120.414061, 20)
vehicle.simple_goto(point2, groundspeed=10)
time.sleep(20)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
time.sleep(20)

print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()