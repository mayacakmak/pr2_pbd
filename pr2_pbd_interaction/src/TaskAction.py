# Manifests
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import time
import sys
import signal
import numpy
from numpy import *
from numpy.linalg import norm
import yaml
import os

# ROS Libraries
import rospy
from pr2_pbd_interaction.msg import *

class TaskAction:
    def __init__(self, type): 
        self.type = type
        if (type.type == TaskActionType.ARM_TARGET):
            self.taskAction = ArmTargetAct()
        elif (type.type == ARM_TRAJECTORY):
            self.taskAction = ArmTargetAct()
        elif (type.type == GRIPPER):
            self.taskAction = GripperAct()   
    
