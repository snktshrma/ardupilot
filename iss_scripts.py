from pymavlink import mavutil
import time
import math

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(master.target_system, master.target_component)

mission_running = False

POSITION_ERROR_LIMIT = 1.0

original_home_coords = (37.7749, -122.4194, 30)

def set_guided_mode():
    master.set_mode('GUIDED')
    print("Switched to GUIDED mode.")

def wait_for_ack(msg_id):
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg and msg.command == msg_id:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Command {msg_id} Acknowledged")
            else:
                print(f"Command {msg_id} Rejected")
            break

def get_drone_position():
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000

def has_reached_target(target_lat, target_lon, target_alt):
    while True:
        pos = get_drone_position()
        if pos:
            lat, lon, alt = pos
            distance = math.sqrt((lat - target_lat)**2 + (lon - target_lon)**2) * 111139
            alt_error = abs(alt - target_alt)

            if distance <= POSITION_ERROR_LIMIT and alt_error <= POSITION_ERROR_LIMIT:
                print(f"Target reached within {distance:.2f}m and {alt_error:.2f}m altitude error.")
                return True
        
        # Check if RTL or Land command has been issued
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            flight_mode = msg.custom_mode
            if flight_mode == 6:  # RTL Mode
                print("RTL detected! Stopping position check.")
                return False
            elif flight_mode == 9:  # Land Mode
                print("Land detected! Stopping position check.")
                return False
        
        time.sleep(1)

def arm_and_takeoff(target_altitude):
    global mission_running
    if mission_running:
        print("Mission already running, aborting new command.")
        return
    mission_running = True

    set_guided_mode()

    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed. Taking off...")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, target_altitude
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    has_reached_target(*get_drone_position()[:2], target_altitude)
    mission_running = False

def move_to_gps(lat, lon, alt):
    global mission_running
    if mission_running:
        print("Mission already running, aborting new command.")
        return
    mission_running = True

    print(f"Moving to {lat}, {lon} at {alt}m altitude")

    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,  # Velocity
        0, 0, 0,  # Acceleration
        0, 0
    )

    has_reached_target(lat, lon, alt)
    mission_running = False

def return_to_home():
    global mission_running
    if mission_running:
        print("Mission already running, aborting new command.")
        return
    mission_running = True

    print("Restoring original home location...")

    master.mav.set_home_position_send(
        master.target_system,
        int(original_home_coords[0] * 1e7),
        int(original_home_coords[1] * 1e7),
        int(original_home_coords[2] * 1000),
        0, 0, 0, 0, 0, 0
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_HOME)

    print("Returning to home...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
    mission_running = False

def rally_point(rally_lat, rally_lon, rally_alt):
    global mission_running
    if mission_running:
        print("Mission already running, aborting new command.")
        return
    mission_running = True

    print(f"Setting new rally point as home: {rally_lat}, {rally_lon}, {rally_alt}")

    master.mav.set_home_position_send(
        master.target_system,
        int(rally_lat * 1e7),
        int(rally_lon * 1e7),
        int(rally_alt * 1000),
        0, 0, 0, 0, 0, 0
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_HOME)

    print("Executing RTL to rally point...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
    mission_running = False

def land():
    global mission_running
    if mission_running:
        print("Mission already running, aborting new command.")
        return
    mission_running = True

    print("Landing now...")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)
    mission_running = False

if __name__ == "__main__":
    rally_coords = (37.7750, -122.4200, 30)

    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 32, 1e6/30, 0,0,0,0,0)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 33, 1e6/30, 0,0,0,0,0)

    arm_and_takeoff(30)
    move_to_gps(37.7750, -122.4199, 30)
    rally_point(*rally_coords)
    return_to_home()
    land()
