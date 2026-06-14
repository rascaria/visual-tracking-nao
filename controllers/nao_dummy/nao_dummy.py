"""
a dummy behavior for a nao robot 
"""

import math
from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

head_yaw   = robot.getDevice("HeadYaw")
head_pitch = robot.getDevice("HeadPitch")

# move arms down
lShoulderPitch = robot.getDevice("LShoulderPitch")
rShoulderPitch = robot.getDevice("RShoulderPitch")
lShoulderPitch.setPosition( math.radians(90) )
rShoulderPitch.setPosition( math.radians(90) )

while robot.step(timestep) != -1:
    t = robot.getTime()
    
    # calculate the target joints
    target_head_yaw   = math.radians(100) * math.sin(t)
    target_head_pitch = math.radians(10) * math.cos(t)
    
    # set joints
    head_yaw.setPosition(target_head_yaw)
    head_pitch.setPosition(target_head_pitch)
