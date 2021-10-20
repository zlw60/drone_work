#!/usr/bin/env python

# Import DroneKit-python
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import  mavutil
import math
import time

# Connect to Vehicle
#import argparse
#parser = argparse.ArgumentParser
#parser.add_argument('--connect', default='127.0.0.1:14550')
#args = parser.parse_args()

#connection_string = "/dev/ttyACM0"
connection_string = "/dev/ttyUSB0"
baud_rate = 57600
print("Connecting to Vehicle on %s" % connection_string)
vehicle = connect(connection_string,baud=baud_rate, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print("Basic Pre-arm Checks")
#    while not vehicle.is_armable:
#        print("Waiting for vehicle to initialize")
#        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("Altitude: ".vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached Target altitude")
            brake
        time.sleep(1)

arm_and_takeoff(5)
print("takeoff complete")
time.sleep(10)

print("Time to land")
vehicle.mode =VehicleMode("LAND")

vehicle.close()
