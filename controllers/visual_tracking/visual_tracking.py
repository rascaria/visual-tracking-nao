"""
 
"""

import math
from controller import Robot, Camera


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


while robot.step(timestep) != -1:
    # current time in s
    t = robot.getTime()
    #print( t )
    
    # save frames (slows down the simulation)
    #camera_top.saveImage('./{}.png'.format(t), 100)
    
    # calculate the target joints
    target_head_yaw   = math.radians(100) * math.sin(t)
    target_head_pitch = math.radians(10) * math.cos(t)
    
    # set joints
    head_yaw.setPosition(target_head_yaw)
    head_pitch.setPosition(target_head_pitch)
