

#!/usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import argparse

def connect_drone(connection_string, baud_rate=57600):
    """Connect to the drone."""
    print(f'Connecting to vehicle on: {connection_string}')
    try:
        vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
        print("Vehicle connected!")
        return vehicle
    except Exception as e:
        print(f"Connection failed: {e}")
        return None

def arm_and_takeoff(vehicle, target_altitude):
    """Arm the drone and takeoff to target altitude."""
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_altitude}")
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    velocity_x: forward/backward velocity (m/s) - positive = forward
    velocity_y: left/right velocity (m/s) - positive = right
    velocity_z: up/down velocity (m/s) - positive = down
    duration: time to maintain the velocity (seconds)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)

    # Send command for duration seconds
    start_time = time.time()
    while time.time() - start_time < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def goto_position_target_local_ned(vehicle, north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (in meters)
        0, 0, 0, # x, y, z velocity in m/s (not used)
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
    vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto_location_global(vehicle, targetLocation):
    """
    Go to a specified global location (latitude, longitude, altitude).
    """
    vehicle.simple_goto(targetLocation)
    
    # Wait until the vehicle reaches target location
    while vehicle.mode.name == "GUIDED":
        current_location = vehicle.location.global_relative_frame
        dist = get_distance_metres(current_location, targetLocation)
        if dist < 1:
            print("Reached target location")
            break
        time.sleep(2)

def print_vehicle_state(vehicle):
    """Print the current state of the vehicle."""
    print(f"Global Location: {vehicle.location.global_frame}")
    print(f"Global Location (relative altitude): {vehicle.location.global_relative_frame}")
    print(f"Local Location: {vehicle.location.local_frame}")
    print(f"Attitude: {vehicle.attitude}")
    print(f"Velocity: {vehicle.velocity}")
    print(f"Battery: {vehicle.battery}")
    print(f"Last Heartbeat: {vehicle.last_heartbeat}")
    print(f"Heading: {vehicle.heading}")
    print(f"Is Armable?: {vehicle.is_armable}")
    print(f"System status: {vehicle.system_status.state}")
    print(f"Mode: {vehicle.mode.name}")

def main():
    parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode')
    parser.add_argument('--connect', help="Vehicle connection target string")
    args = parser.parse_args()

    connection_string = args.connect if args.connect else '/dev/ttyAMA0'
    
    # Connect to the Vehicle
    vehicle = connect_drone(connection_string)
    if vehicle is None:
        return

    try:
        # Print vehicle state
        print_vehicle_state(vehicle)

        # Arm and takeoff to 10 meters
        arm_and_takeoff(vehicle, 10)

        # Example movement commands
        print("Moving forward at 5 m/s for 5 seconds")
        send_ned_velocity(vehicle, 5, 0, 0, 5)
        
        print("Moving right at 5 m/s for 5 seconds")
        send_ned_velocity(vehicle, 0, 5, 0, 5)

        # Example of going to a specific local position
        print("Going to position 10 meters North, 10 meters East, -10 meters Down")
        goto_position_target_local_ned(vehicle, 10, 10, -10)
        time.sleep(10)

        # Return to launch
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")

        # Wait until landed and disarmed
        while vehicle.armed:
            print(" Waiting for disarming...")
            time.sleep(1)

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Close vehicle object
        print("Close vehicle object")
        vehicle.close()

if __name__ == '__main__':
    main()

