# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 23:51:00 2018

@author: drkfr
"""

#!/usr/bin/python
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('tcp:127.0.0.1:5760', wait_ready=False)
print "Arming motors:"

while not vehicle.is_armable:
    time.sleep
    
vehicle.mode    = VehicleMode("GUIDED")
vehicle.armed   = True

while not vehicle.armed:
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    print "  Waiting for arming to be finished"
    time.sleep(1)
        
print "Keeping motors armed for 5s"
time.sleep(2)

print "Disarming"
vehicle.armed   = False

while vehicle.armed:
        print "  Waiting for disarm"
        time.sleep(1)

vehicle.close()