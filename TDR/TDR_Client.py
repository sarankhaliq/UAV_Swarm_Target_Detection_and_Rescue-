import socket
from pymavlink import mavutil
import json
import time

# Mavlink_connection
the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

def mod_gps(uav1):
    while True:
        msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            lat = msg.lat / 10**7
            lon = msg.lon / 10**7
            alt = msg.relative_alt * 0.001
            print("Current lat = ", lat)
            return lat, lon, alt

def send_to_waypoint(uav1, lat, lon, alt):
    msg = uav1.mav.set_position_target_global_int_encode(
        0,      # time_boot_ms (not used)
        0, 0,   # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b110111111000, # type_mask (only positions enabled)
        int(lat * 10**7), int(lon * 10**7), 40,  # x, y, z positions
        0, 0, 0,    # x, y, z velocity in m/s (not used)
        0, 0, 0,    # x, y, z acceleration (not used)
        0, 0)       # yaw, yaw_rate (not used)
    uav1.mav.send(msg)

def start_mavlink_client():
    try:
        # Create a UDP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.setblocking(False)

        # Server address and port
        host = '192.168.1.83'
        port = 12345

        

        previous_waypoint = None

        while True:
            # Get GPS data
            lat1, lon1, alt1 = mod_gps(the_connection)
            print('Socket Created')
            #data = {'lat': lat1, 'lon': lon1, 'alt': alt1}
            data = "Connected"
            gps = json.dumps(data).encode('utf-8')

            # Send the GPS data to the server
            client_socket.sendto(gps, (host, port))
            print('Data Sent')

            # Receive acknowledgment from the server
            try:
                data, server = client_socket.recvfrom(1024)
                rcv_data = json.loads(data.decode('utf-8'))
                alt = rcv_data['alt']
                lon = rcv_data['lon']
                lat = rcv_data['lat']
                print("Data Received")
                print([lat, lon, alt])
	
	            # Check if the received waypoint is different from the previous one
                current_waypoint = (lat, lon, alt)
                if current_waypoint != previous_waypoint:
	                # Move to the received GPS waypoint
                    send_to_waypoint(the_connection, lat, lon, alt)
                    print("Moving to waypoint")
                    previous_waypoint = current_waypoint
                else:
	                print("Waypoint is the same as the previous one. Not moving.")
                    #time.sleep(1)
            except:
	            print("No Data Received")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        client_socket.close()

if __name__ == '__main__':
    start_mavlink_client()