from pymavlink import mavutil
import time

# Connect to the MAVLink device (replace with your connection string)
# Example for serial: '/dev/ttyUSB0', baud=115200
# Example for UDP: 'udp:127.0.0.1:14550'
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for a heartbeat from the autopilot to ensure connection is established
master.wait_heartbeat()
print("MAVLink connection established")

# Function to send an Optical Flow message
def send_optical_flow():
    master.mav.optical_flow_send(
        int(time.time() * 1e6),  # time_usec (current time in microseconds)
        0,  # sensor_id (example: 0 for primary sensor)
        100,  # flow_x (pixel displacement in X)
        50,   # flow_y (pixel displacement in Y)
        0.05,  # flow_comp_m_x (computed flow in meters/sec)
        -0.03, # flow_comp_m_y (computed flow in meters/sec)
        200,   # quality (optical flow quality)
        10    # ground_distance (meters)
    )
    print("OPTICAL_FLOW message sent!")

# Send the message in a loop (e.g., every 100ms)
while True:
    send_optical_flow()
    time.sleep(0.1)  # Send every 100ms
