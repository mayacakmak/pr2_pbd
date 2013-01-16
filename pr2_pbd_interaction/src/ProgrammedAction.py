# Manifests
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('visualization_msgs')

# Generic libraries
import time
import sys
import signal
import numpy
from numpy import *
from numpy.linalg import norm
import os
from geometry_msgs.msg import *
from visualization_msgs.msg import MarkerArray, Marker

# ROS Libraries
import rospy
import rosbag
from pr2_pbd_interaction.msg import *
from ActionStepMarker import *
from std_msgs.msg import Header,ColorRGBA

class ProgrammedAction:
    
    def __init__(self, skillIndex):
        self.seq = ActionStepSequence()
        self.skillIndex = skillIndex
        self.rMarkers = []
        self.lMarkers = []
        self.rLinks = dict()
        self.lLinks = dict()
        self.markerOutput = rospy.Publisher('visualization_marker_array', MarkerArray)
    
    def copy(self):
        pAction = ProgrammedAction(self.skillIndex)
        pAction.seq = ActionStepSequence()
        for i in range(len(self.seq.seq)):
            aStep = self.seq.seq[i]
            aStepCopy = self.copyActionStep(aStep)
            pAction.seq.seq.append(aStepCopy)
        return pAction

    def copyActionStep(self, aStep):
        aStepCopy = ActionStep()
        aStepCopy.type = int(aStep.type)
        if (aStepCopy.type == ActionStep.ARM_TARGET):
            aStepCopy.armTarget = ArmTarget()
            aStepCopy.armTarget.rArmVelocity = float(aStep.armTarget.rArmVelocity)
            aStepCopy.armTarget.lArmVelocity = float(aStep.armTarget.lArmVelocity)
            aStepCopy.armTarget.rArm = self.copyArmState(aStep.armTarget.rArm)
            aStepCopy.armTarget.lArm = self.copyArmState(aStep.armTarget.lArm)
        elif (aStepCopy.type == ActionStep.ARM_TRAJECTORY):
            aStepCopy.armTrajectory = ArmTrajectory()
            aStepCopy.armTrajectory.timing = aStep.armTrajectory.timing[:]
            for j in range(len(aStep.armTrajectory.timing)):
                aStepCopy.armTrajectory.rArm.append(self.copyArmState(aStep.armTrajectory.rArm[j]))
                aStepCopy.armTrajectory.lArm.append(self.copyArmState(aStep.armTrajectory.lArm[j]))
            aStepCopy.armTrajectory.rRefFrame = int(aStep.armTrajectory.rRefFrame)
            aStepCopy.armTrajectory.lRefFrame = int(aStep.armTrajectory.lRefFrame)
        elif(aStepCopy.type == ActionStep.GRIPPER_TARGET):
            aStepCopy.gripperTarget = GripperTarget()
            aStepCopy.gripperTarget.rGripper = GripperState(aStep.gripperTarget.rGripper.state)
            aStepCopy.gripperTarget.lGripper = GripperState(aStep.gripperTarget.lGripper.state)
        return aStepCopy
    
    def copyArmState(self, armState):
        armStateCopy = ArmState()
        armStateCopy.refFrame = int(armState.refFrame)
        armStateCopy.joint_pose = array(armState.joint_pose)
        armStateCopy.ee_pose = Pose(armState.ee_pose.position, armState.ee_pose.orientation)
        return armStateCopy
     
    def getName(self):
        return 'Action' + str(self.skillIndex)

    def addActionStep(self, step):
        self.seq.seq.append(self.copyActionStep(step))
        
        if (step.type == ActionStep.ARM_TARGET or step.type == ActionStep.ARM_TRAJECTORY):
            self.rMarkers.append(ActionStepMarker(self.nFrames(), 0, self.getLastStep()))
            self.lMarkers.append(ActionStepMarker(self.nFrames(), 1, self.getLastStep()))
            if (self.nFrames() > 1):
                self.rLinks[self.nFrames()-1] = self.getLink(0, self.nFrames()-1)
                self.lLinks[self.nFrames()-1] = self.getLink(1, self.nFrames()-1)
            
    def getLink(self, armIndex, toIndex):
        if (armIndex == 0):
            startPoint = self.rMarkers[toIndex-1].getAbsolutePosition(isStart=True)
            endPoint = self.rMarkers[toIndex].getAbsolutePosition(isStart=False)
        else:
            startPoint = self.lMarkers[toIndex-1].getAbsolutePosition(isStart=True)
            endPoint = self.lMarkers[toIndex].getAbsolutePosition(isStart=False)
            
        return Marker(type=Marker.ARROW, id=(2*toIndex+armIndex), lifetime=rospy.Duration(2),
                      scale=Vector3(0.01,0.03,0.01), header=Header(frame_id='base_link'),
                      color=ColorRGBA(0.8, 0.8, 0.8, 0.3), points=[startPoint, endPoint])

    def updateInteractiveMarkers(self):
        for i in range(len(self.rMarkers)):
            self.rMarkers[i].updateVisualization()
        for i in range(len(self.lMarkers)):
            self.lMarkers[i].updateVisualization()

    def updateLinks(self):
        for i in self.rLinks.keys():
            self.rLinks[i] = self.getLink(0, i)
        for i in self.lLinks.keys():
            self.lLinks[i] = self.getLink(1, i)
            
    def updateVisualization(self):
        mArray = MarkerArray()
        for i in self.rLinks.keys():
            mArray.markers.append(self.rLinks[i])
        for i in self.lLinks.keys():
            mArray.markers.append(self.lLinks[i])
        self.markerOutput.publish(mArray)
        self.updateLinks()
        
    def clear(self):
        #TODO: get backups before clear
        self.seq = ActionStepSequence()
        self.rMarkers = []
        self.lMarkers = []
        self.rLinks = dict()
        self.lLinks = dict()

    def undoClear(self):
        self.seq = [] #TODO
        
    def getFname(self, ext='.bag'):
        if ext[0] != '.' :
            ext  = '.' + ext
        return self.getName() + ext
    
    def nFrames(self):
        return len(self.seq.seq)
    
    def save(self, dataDir):
        if (self.nFrames() > 0):
            demoBag = rosbag.Bag(dataDir + self.getFname(), 'w')
            demoBag.write('sequence', self.seq)
            demoBag.close()
        else:
            rospy.logwarn('Could not save demonstration because it does not have any frames.')
        
    def load(self, dataDir):
        fname = dataDir + self.getFname()
        if (os.path.exists(fname)):
            demoBag = rosbag.Bag(fname, 'r')
            self.seq = demoBag.read('sequence')
            demoBag.close()
        else:
            rospy.logwarn('File does not exist, cannot load demonstration: '+ fname)
    
    def getLastStep(self):
        return self.seq.seq[len(self.seq.seq)-1]
    
    def deleteLastStep(self):
        # TODO: backup last pose for undo
        self.seq.seq = self.seq.seq[0:len(self.seq.seq)-1]
        
    def resumeDeletedPose(self):
        self.seq.seq.append(None) #TODO

    ## TODO??        
    def getTrajectory(self):
        return self.seq
    
    def requiresObject(self):
        for i in range(len(self.seq.seq)):
            if ((self.seq.seq[i].type == ActionStep.ARM_TARGET and 
                (self.seq.seq[i].armTarget.rArm.refFrame == ArmState.OBJECT or 
                 self.seq.seq[i].armTarget.lArm.refFrame == ArmState.OBJECT)) or 
                (self.seq.seq[i].type == ActionStep.ARM_TRAJECTORY and 
                (self.seq.seq[i].armTrajectory.rRefFrame == ArmState.OBJECT or 
                 self.seq.seq[i].armTrajectory.lRefFrame == ArmState.OBJECT))):
                return True
        return False

    def getStep(self, index):
        return self.seq.seq[index]
