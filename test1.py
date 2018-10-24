# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 23:13:11 2018

@author: drkfr
"""

from dronekit import connect, VehicleMode
import time

# Connect to the vehicle
vehicle = connect('tcp:127.0.0.1:5760',wait_ready=True)

# vehicle is an instance of the Vehicle class
print "Autopilot Firmware version: %s" % vehicle.version
print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
print "Global Location: %s" % vehicle.location.global_frame
print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print "Local Location: %s" % vehicle.location.local_frame    #NED
print "Attitude: %s" % vehicle.attitude
print "Velocity: %s" % vehicle.velocity
print "GPS: %s" % vehicle.gps_0
print "Groundspeed: %s" % vehicle.groundspeed
print "Airspeed: %s" % vehicle.airspeed
print "Gimbal status: %s" % vehicle.gimbal
print "Battery: %s" % vehicle.battery
print "EKF OK?: %s" % vehicle.ekf_ok
print "Last Heartbeat: %s" % vehicle.last_heartbeat
print "Rangefinder: %s" % vehicle.rangefinder
print "Rangefinder distance: %s" % vehicle.rangefinder.distance
print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print "Heading: %s" % vehicle.heading
print "Is Armable?: %s" % vehicle.is_armable
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name    # settable
print "Armed: %s" % vehicle.armed    # settable


vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
#
while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print " Getting ready to take off ..."
    time.sleep(1)
    
if vehicle.armed == True:
    time.sleep(5)
    
vehicle.close()