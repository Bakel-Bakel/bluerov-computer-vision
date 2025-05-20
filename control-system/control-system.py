#!/usr/bin/env python
# script example to operate the bluerow using python

import gii_bluerov
import dronekit
from time import sleep
import cv2
import numpy as np
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# Connect to the Vehicle autopilot.
connection_string="192.168.3.10:14551"
print("Connecting to vehicle on: %s" % (connection_string,))
autopilot = dronekit.connect(connection_string, wait_ready=False)

# Get some vehicle attributes (state)
print (" Battery: %s" % autopilot.battery)

# Arm to move the robot
autopilot.armed=True
sleep(5.0)
print ("%s" % autopilot.armed)
print(autopilot.location.global_relative_frame.alt)

# PID Constants for depth control
Kp_depth = 100
Ki_depth = 25
Kd_depth = 0
depth_setpoint = -0.4  # Desired depth (e.g., 1 meter)
dt = 0.1      # Time step in seconds

# PID Constants for distance control
Kp_dist = 0.5
Ki_dist = 0.1
Kd_dist = 0.05
distance_setpoint = 0.1  # Desired distance in meters (10cm)
distance_dt = 0.1

# PID Variables for depth
depth_previous_error = 0
depth_integral = 0
current_depth = autopilot.location.global_relative_frame.alt  # Initial depth

# PID Variables for distance
distance_previous_error = 0
distance_integral = 0
current_distance = 0

# Video capture setup
port = 5600
video_source = f'udpsrc port={port}'
video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

# Create GStreamer pipeline
video_pipe = Gst.parse_launch(' '.join([video_source, video_codec, video_decode, video_sink_conf]))
video_pipe.set_state(Gst.State.PLAYING)
video_sink = video_pipe.get_by_name('appsink0')

def gst_to_opencv(sample):
    """Transform byte array into np array"""
    buf = sample.get_buffer()
    caps_structure = sample.get_caps().get_structure(0)
    array = np.ndarray(
        (
            caps_structure.get_value('height'),
            caps_structure.get_value('width'),
            3
        ),
        buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
    return array

def calculate_distance(area):
    """Calculate distance based on area using the given formula"""
    if area > 0:
        return 1329.34 * (area ** (-0.3326))
    return float('inf')

def detect_yellow_color(frame):
    """Detect yellow color and return center coordinates and area"""
    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Yellow color range in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    total_area = 0
    weighted_sum_x = 0
    weighted_sum_y = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter out small contours
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Accumulate weighted sums
                weighted_sum_x += cX * area
                weighted_sum_y += cY * area
                total_area += area
                
                # Draw the contour and centroid on the frame
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                
                # Display area for each contour
                cv2.putText(frame, f"Area: {int(area)}", (cX - 20, cY - 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Calculate weighted center if any yellow was detected
    if total_area > 0:
        weighted_avg_x = weighted_sum_x / total_area
        weighted_avg_y = weighted_sum_y / total_area
        weighted_center = (int(weighted_avg_x), int(weighted_avg_y))
        
        # Draw the weighted center on the frame
        cv2.circle(frame, weighted_center, 7, (0, 0, 255), -1)
        
        # Draw the image center
        image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        cv2.circle(frame, image_center, 5, (255, 0, 0), -1)
        
        # Calculate and display distance
        distance = calculate_distance(total_area)
        
        # Display total area and distance at the top of the frame
        cv2.putText(frame, f"Total Area: {int(total_area)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, f"Distance: {distance:.2f}m", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return weighted_center, total_area, distance
    
    return None, 0, float('inf')

def compute_depth_control(current_depth):
    """Compute PID control for depth"""
    global depth_previous_error, depth_integral
    
    error = depth_setpoint - current_depth
    depth_integral += error * dt
    derivative = (error - depth_previous_error) / dt
    
    output = (Kp_depth * error) + (Ki_depth * depth_integral) + (Kd_depth * derivative)
    depth_previous_error = error
    
    return round(output)

def compute_distance_control(current_distance):
    """Compute PID control for distance"""
    global distance_previous_error, distance_integral
    
    error = distance_setpoint - current_distance
    distance_integral += error * distance_dt
    derivative = (error - distance_previous_error) / distance_dt
    
    output = (Kp_dist * error) + (Ki_dist * distance_integral) + (Kd_dist * derivative)
    distance_previous_error = error
    
    # Limit the output to reasonable values
    output = max(min(output, 100), -100)
    return round(output)

def main():
    try:
        while True:
            # Get video frame
            sample = video_sink.emit('pull-sample')
            if sample:
                frame = gst_to_opencv(sample)
                
                # Detect yellow color and get distance
                yellow_center, yellow_area, current_distance = detect_yellow_color(frame)
                
                # Get current depth
                current_depth = autopilot.location.global_relative_frame.alt
                
                # Compute depth control
                depth_control = compute_depth_control(current_depth)
                
                # Apply depth control
                gii_bluerov.move_rov(autopilot, "z", "displacement", depth_control)
                
                # If yellow is detected, control distance
                if yellow_center is not None:
                    distance_control = compute_distance_control(current_distance)
                    # Use x-axis for forward/backward movement
                    gii_bluerov.move_rov(autopilot, "x", "displacement", distance_control)
                else:
                    # Stop forward/backward movement if no yellow detected
                    gii_bluerov.move_rov(autopilot, "x", "displacement", 0)
                
                # Print status
                print(f"Depth: {current_depth:.3f}m | Depth Control: {depth_control:.3f}")
                if yellow_center:
                    print(f"Distance: {current_distance:.3f}m | Distance Control: {distance_control:.3f}")
                
                # Display frame
                cv2.imshow('Frame', frame)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
            sleep(0.1)  # Small delay to prevent CPU overload
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Cleanup
        gii_bluerov.stop_rov(autopilot)
        autopilot.armed = False
        video_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()