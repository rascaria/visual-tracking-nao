"""
 
"""

import math
from controller import Robot, Camera
import cv2
import numpy as np


robot = Robot()

# we process each image every 40ms = 25fps
timestep = int(robot.getBasicTimeStep() * 4)

# get access to the cameras and enable them
camera_top    = robot.getDevice("CameraTop")
camera_top.enable(timestep)

## enable the bottom camera if necessary
#camera_bottom = robot.getDevice("CameraBottom")
#camera_bottom.enable(timestep)

head_yaw   = robot.getDevice("HeadYaw")
head_pitch = robot.getDevice("HeadPitch")

# move arms down 
lShoulderPitch = robot.getDevice("LShoulderPitch")
rShoulderPitch = robot.getDevice("RShoulderPitch")
lShoulderPitch.setPosition( math.radians(90) )
rShoulderPitch.setPosition( math.radians(90) )

# Get the dimensions of the camera image
width = camera_top.getWidth()
height = camera_top.getHeight()

# P-Controller constants and initial target angles
Kp = 0.0003
target_yaw = 0.0
target_pitch = 0.0
search_direction = 1
search_speed = 0.02

while robot.step(timestep) != -1:
    # Read the image from the NAO's top camera
    raw_img = camera_top.getImage()
    
    if raw_img:
        # Convert the raw Webots image to an OpenCV numpy array
        img = np.frombuffer(raw_img, np.uint8).reshape((height, width, 4))
        
        # Drop the alpha channel
        img = img[:, :, [0, 1, 2]]
        
        # Temporary Picture for color picker
        # cv2.imwrite('nao_camera_view.png', img)
        
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the color yellow
        lower_yellow = np.array([15, 120, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Apply basic morphological operations (erode and dilate) to clean the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        duck_found = False
        
        if contours:
            # Filter contours by shape (aspect ratio) and size (minimum area)
            valid_contours = []
            for c in contours:
                area = cv2.contourArea(c)
                if area > 100: # minimum area threshold
                    x, y, w, h = cv2.boundingRect(c)
                    aspect_ratio = w / float(h) if h > 0 else 0
                    if 0.75 <= aspect_ratio <= 1.35:
                        valid_contours.append(c)
            
            if valid_contours:
                duck_found = True
                # Find the lowest contour in the camera image (highest Y-coordinate for the bottom edge)
                lowest_contour = max(valid_contours, key=cv2.contourArea)
                
                # Calculate its center (X, Y pixel coordinates) using image moments
                M = cv2.moments(lowest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Calculate the error from the center of the image
                    error_x = (width / 2) - cx
                    error_y = cy - (height / 2) # Positive when duck is lower in the image
                    
                    # Update motor positions using the Proportional Controller
                    if abs(error_x) > 15:
                        target_yaw += Kp * error_x
                    if abs(error_y) > 15:
                        target_pitch += Kp * error_y
                    
        # If no duck was found, enter Search Mode
        if not duck_found:
            target_yaw += search_speed * search_direction
            if target_yaw >= 2.0:
                search_direction = -1
            elif target_yaw <= -2.0:
                search_direction = 1
                
        # Clamp the motor values to prevent exceeding NAO's neck physical limits
        target_yaw = max(-2.0, min(2.0, target_yaw))
        target_pitch = max(-0.6, min(0.5, target_pitch))
        
        # Apply the calculated positions to the motors
        head_yaw.setPosition(target_yaw)
        head_pitch.setPosition(target_pitch)
