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
connection_string="192.168.3.20:14552"
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
Kp = 100
Ki = 25
Kd = 0
setpoint = -1  # Desired depth (e.g., 1 meter)
dt = 0.1      # Time step in seconds

# PID Variables
previous_error = 0
integral = 0
current_depth = autopilot.location.global_relative_frame.alt  # Initial depth

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
    
    # Calculate weighted center if any yellow was detected
    if total_area > 0:
        weighted_avg_x = weighted_sum_x / total_area
        weighted_avg_y = weighted_sum_y / total_area
        return (int(weighted_avg_x), int(weighted_avg_y)), total_area
    
    return None, 0

def compute_depth_control(current_depth):
    """Compute PID control for depth"""
    global previous_error, integral
    
    error = setpoint - current_depth
    integral += error * dt
    derivative = (error - previous_error) / dt
    
    output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    previous_error = error
    
    return round(output)

def main():
    try:
        while True:
            # Get video frame
            sample = video_sink.emit('pull-sample')
            if sample:
                frame = gst_to_opencv(sample)
                
                # Detect yellow color
                yellow_center, yellow_area = detect_yellow_color(frame)
                
                # Get current depth
                current_depth = autopilot.location.global_relative_frame.alt
                
                # Compute depth control
                depth_control = compute_depth_control(current_depth)
                
                # Apply depth control
                gii_bluerov.move_rov(autopilot, "z", "displacement", depth_control)
                
                # Print status
                print(f"Depth: {current_depth:.3f}m | Control: {depth_control:.3f}")
                if yellow_center:
                    print(f"Yellow detected at {yellow_center} with area: {yellow_area}")
                
                # Display frame (optional)
                cv2.imshow('Frame', frame)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
            sleep(0.1)  # Small delay to prevent CPU overload
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Cleanup
        autopilot.armed = False
        video_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()