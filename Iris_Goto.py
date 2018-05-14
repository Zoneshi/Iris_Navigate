#!/usr/bin/env python
"""
This is for Discover lab Iris+ Drone mission planning
Author: Zone Shi
Last Modified: May, 4, 2018
"""
import dronekit
import exceptions
import socket
import time
from pymavlink import mavutil
import math

def Iris_Connnect(pi_serial, pi_rate):

    """connect with ardupilot via serial port"""

    try:
        """For Rasepberry pi """
        Iris = dronekit.connect(pi_serial, wait_ready=True, baud=pi_rate)

        """For debugging with DroneKit-sitl"""
        #Iris = dronekit.connect('tcp:127.0.0.1:5760',wait_ready=True)

        """For debugging with sim_vehicle"""
        #Iris = dronekit.connect('127.0.0.1:14550',wait_ready=True)
    # Bad TCP connection
    except socket.error:
        print('No server exists!')

    # Bad TTY connection
    except exceptions.OSError as e:
        print('No serial exists!')

    # API Error
    except dronekit.APIException:
        print('Timeout!')

    # Other error
    except:
        print('Some other error!')

    return Iris

def arm_and_takeoff(vehicle, targetAlt = 1):
    """
    Arms vehicle and fly to a TargetAltitude.
    """

    print("Pre-arm checks")
    #Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
       print(" Waiting for vehicle to initialise...")
       time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    vehicle.mode = dronekit.VehicleMode("GUIDED")

    print("Taking off!")
    vehicle.simple_takeoff(targetAlt) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAlt*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def goto_position_target_global_int(vehicle,aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto_position_target_local_ned(vehicle,north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    It is important to remember that in this frame, positive altitudes are entered as negative
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)


if __name__ == "__main__":

    pi_serial_addr = '/dev/ttyAMA0'
    pi_serial_rate = 57600

    Iris_Vehicle = Iris_Connnect(pi_serial_addr,pi_serial_rate)

    #Change to GUIDED mode and takeoff to target altitude
    arm_and_takeoff(Iris_Vehicle,targetAlt = 10)
    i = 0;
    while True:
        print("Relative Waypoint (North, East, Down)# ",i)
        go_to_lo = input()
        goto_position_target_local_ned(Iris_Vehicle,go_to_lo[0],go_to_lo[1],go_to_lo[2])
        i +=1
