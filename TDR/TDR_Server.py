import sys
import cv2 
import math 
import imutils
import json
import socket
from pymavlink import mavutil
import time
from time import sleep, strftime, time

from ultralytics import YOLO


########### Functions
def send_to_waypoint(uav1, lat, lon, alt):
    msg = uav1.mav.set_position_target_global_int_encode(
        0,      # time_boot_ms (not used)
        0, 0,   # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b110111111000, # type_mask (only positions enabled)
        int(lat * 10**7), int(lon * 10**7), 20,  # x, y, z positions
        0, 0, 0,    # x, y, z velocity in m/s (not used)
        0, 0, 0,    # x, y, z acceleration (not used)
        0, 0)       # yaw, yaw_rate (not used)
    uav1.mav.send(msg)


def get_rel_alt(connection):  # Getting relative altitude of bird
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        Rel_alt_vec = the_connection.messages['GLOBAL_POSITION_INT'].relative_alt/1000

    return Rel_alt_vec
    
def calculate_uav_distances(image_center, bbox_center , altitude, fov_angle_full, horizontal_width):
   # Calculate Horizental_oi
    h_dist = bbox_center[0] - image_center[0]
    v_dist = -(bbox_center[1] - image_center[1])    
    
    # Calculate the half angle of field of view and half the width of the image
    theta = fov_angle_full / 2

    # Convert the angle to radians for the tangent function
    theta_rad = math.radians(theta)

    # Calculate the base (half the width of the field of view in meters) using the tangent function
    half_base = altitude * math.tan(theta_rad)

    # Since we have half of the base, multiply by 2 to get the full base width
    full_base = half_base * 2

    # Calculate the pixel-to-meter ratio
    pixel_to_meter_ratio = full_base / horizontal_width
    print(" " )

    # Calculate the UAV distances
    uav_h_dist = h_dist * pixel_to_meter_ratio
    uav_v_dist = v_dist * pixel_to_meter_ratio

    return uav_h_dist, uav_v_dist

def mod_gps(uav1):
    msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 10**7
    lon = msg.lon / 10**7
    alt = msg.relative_alt * 0.001
    return lat, lon, alt


########### Load Model and Get camera Stream
model = YOLO("yolov8s.engine", task="detect")
print("Model Loaded")

gst_str = (
		'rtspsrc location=rtsp://192.168.1.23:8554/main.264 latency =0 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! appsink drop=True max-buffers=1')

# Create a VideoCapture object with GStreamer pipeline
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
sleep(2)

if not cap.isOpened():
    print("Error: Unable to open RTSP stream. SYSTEM EXIT")
    sys.exit()
    
frame_width = int(cap.get(3)) 
frame_height = int(cap.get(4)) 
   
size = (640, 360) 
result = cv2.VideoWriter('result.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         25, size) 



############## UAV CODE
lat, lon, alt = 33.771094, 72.823122, 20
# Start a connection listening to a UDP port
# Initialize MAVLink connection
the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

send_to_waypoint(the_connection, lat, lon, alt)
the_connection.recv_match(type='COMMAND_ACK', blocking=False)  
                                            
###### Detection Code
fov = 81
detect_count = 0
target_detected = False
sleep(1)
try:
    #Create a UDP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Define server address and port
    host = ''
    port = 12345
    # Bind socket to address and port
    server_socket.bind((host, port))
    print(f'Server listening on {host}:{port}')

    while True:
         
        grabbed=cap.grab()
        ret, frame = cap.retrieve()
        print(ret)
        start_time = time()
        data, addr = server_socket.recvfrom(1024)
        print(f"Received data from {addr}, and data is {data}")

            # Fetch GPS data using MAVLink connection

        
        if ret:
            timestamp = strftime("%Y%m%d_%H%M%S")
            frame = imutils.resize(frame, width=640)
            height, width, _ = frame.shape
            cx, cy = 640/2, 360/2
            image_center = (cx, cy)
            detections = model.predict(frame, save= True)

            if detections:
               #print("Length of detections:", len(detections))
                detc = detections[0]

                try:    
                    for obj in detc.boxes: 
                        class_id = obj.cls.item()
                        #print("class id:", class_id)
                              

                        if int(class_id) == 2 :
                            print("Class id Found")
                            box = obj.xyxy.tolist()
                            x1 = int(box[0][0])
                            y1 = int(box[0][1])
                            x2 = int(box[0][2])
                            y2 = int(box[0][3])
                            color = (0,0,255) 
                            cv2.rectangle(frame, (x1,y1),(x2,y2),color,2)
                            cv2.putText(frame, str(class_id),(x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 2, color,2)
                            obj_cx, obj_cy = x1 + (x2-x1)/2, y1 + (y2 -y1)/2
                            bbox_center = (obj_cx,obj_cy)                        
                            detect_count=detect_count+1
                            
                            print("Number of detection: ", detect_count)
                            if detect_count>=1 and target_detected == False :
                                print("Setting velocity to 0")
                                the_connection.mav.send(
                                mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                                10, 
                                the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_FRAME_BODY_NED,
                                int(0b010111111000),
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))  
                            	## THis command to wait for zero velocity
                                the_connection.recv_match(type='COMMAND_ACK', blocking=False)  
                                sleep(1)
                                if detect_count >= 3:
                                
                                    alt=get_rel_alt(the_connection)
                                    sleep(1)
                                    print("Altitude: ", alt)                      
                                    y, x = calculate_uav_distances(image_center, bbox_center, alt, fov, width)                  
                                    print("Go to the (",y,",",x, ") Body Offset Position")
                                    the_connection.mav.send(
                                     mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                                     10,the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                                     int(0b010111111000),
                                     x, y, 0, # position
                                     0, 0, 0, # velocity
                                     0, 0, 0, # acceleration
                                     0, 0))   # yaw, yaw_speed
                                    target_detected == True
                                    lat, lon, alt = mod_gps(the_connection)
                                    waypoint_data = {'lat': lat, 'lon': lon, 'alt': alt}
                                    gps = json.dumps(waypoint_data).encode('utf-8')
                                    server_socket.sendto(gps, addr)
                                    print(f"Sent target waypoint data to {addr}, lat = {lat},lon = {lon}")
                                                                   
                            for _ in range(3):
                                result.write(frame)                                                    
                            else:
                        	    print("Class id not found")
                        	    result.write(frame)
                         
                except Exception as e:  
                    result.write(frame) 
                    print(e)
                    print("Exception in detections")  
                    print("No Detection! Video Write Continue")              
        else:
            result.write(frame)  
            print("No Detection! Video Write Continue")  

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    server_socket.close()

cap.set(cv2.CAP_GSTREAMER, 0)
cap.release()
result.release()