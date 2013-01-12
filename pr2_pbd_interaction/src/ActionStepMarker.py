# Manifests
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('interactive_markers')
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import time
import sys
import signal
import numpy
from numpy import *
from numpy.linalg import norm
import os
from geometry_msgs.msg import *

# ROS Libraries
import rospy
import rosbag
from pr2_pbd_interaction.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from Arms import *

class ActionStepMarker:
    
    IMServer = None

    def __init__(self, id, armIndex, aStep):
        
        if ActionStepMarker.IMServer == None:
            ActionStepMarker.IMServer = InteractiveMarkerServer('menu')

        self.refNames = ['ROBOT_BASE', 'ROBOT_OTHER_ARM', 'OBJECT', 'PREVIOUS_TARGET', 'NOT_MOVING']
        self.aStep = aStep
        self.armIndex = armIndex
        self.id = 2*id + self.armIndex
        self.name = 'step' + str(id) + 'arm' + str(self.armIndex)

        if (self.getReferenceFrame() != ArmState.NOT_MOVING):
            self.menuHandler = MenuHandler()
            frameEntry = self.menuHandler.insert('Reference frame')
            self.subEntries = [None]*len(self.refNames)
            for i in range(len(self.refNames)):
                self.subEntries[i] = self.menuHandler.insert(self.refNames[i], parent=frameEntry, callback=self.changeReferenceFrame)
    
            for i in range(len(self.refNames)):
                self.menuHandler.setCheckState(self.subEntries[i], MenuHandler.UNCHECKED)
            self.menuHandler.setCheckState(self.subEntries[self.getReferenceFrame()], MenuHandler.CHECKED)
            
            self.updateVisualization()
            self.menuHandler.apply(ActionStepMarker.IMServer, self.name)
            ActionStepMarker.IMServer.applyChanges()

    def getReferenceFrame(self):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.aStep.armTarget.rArm.refFrame
            else:
                return self.aStep.armTarget.lArm.refFrame
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                return self.aStep.armTrajectory.rRefFrame
            else:
                return self.aStep.armTrajectory.lRefFrame
        else:
            rospy.logerr('Unhandled marker type: ' + str(self.aStep.type))
        
    def setReferenceFrame(self, newRef):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                self.aStep.armTarget.rArm = World.convertRefFrame(newRef, self.aStep.armTarget.rArm)
            else:
                self.aStep.armTarget.lArm = World.convertRefFrame(newRef, self.aStep.armTarget.lArm)
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            for i in range(len(self.aStep.armTrajectory.timing)):
                if self.armIndex == 0:
                    self.aStep.armTrajectory.rArm[i] = World.convertRefFrame(newRef, self.aStep.armTrajectory.rArm[i])
                else:
                    self.aStep.armTrajectory.lArm[i] = World.convertRefFrame(newRef, self.aStep.armTrajectory.lArm[i])
            if self.armIndex == 0:
                self.aStep.armTrajectory.rRefFrame = newRef
            else:
                self.aStep.armTrajectory.lRefFrame = newRef

    def getFrameAbsolutePosition(self, armState):
        if (armState.refFrame == ArmState.OBJECT):
            armStateCopy = ArmState(armState.refFrame, 
                                    Pose(armState.ee_pose.position, armState.ee_pose.orientation), 
                                    armState.joint_pose[:])
            World.convertRefFrame(ArmState.ROBOT_BASE, armStateCopy)
            return armStateCopy.ee_pose.position
        else:
            return armState.ee_pose.position
            
    def getAbsolutePosition(self, isStart=True):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.getFrameAbsolutePosition(self.aStep.armTarget.rArm)
            else:
                return self.getFrameAbsolutePosition(self.aStep.armTarget.lArm)
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                if isStart:
                    return self.getFrameAbsolutePosition(self.aStep.armTrajectory.rArm[len(self.aStep.armTrajectory.rArm)-1])
                else:
                    return self.getFrameAbsolutePosition(self.aStep.armTrajectory.rArm[0])
            else:
                if isStart:
                    return self.getFrameAbsolutePosition(self.aStep.armTrajectory.lArm[len(self.aStep.armTrajectory.rArm)-1])
                else:
                    return self.getFrameAbsolutePosition(self.aStep.armTrajectory.lArm[0])
                        
    def getPose(self):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.aStep.armTarget.rArm.ee_pose
            else:
                return self.aStep.armTarget.lArm.ee_pose
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                return self.aStep.armTrajectory.rArm[int(len(self.aStep.armTrajectory.rArm)/2)].ee_pose
            else:
                return self.aStep.armTrajectory.lArm[int(len(self.aStep.armTrajectory.lArm)/2)].ee_pose

    def getTrajPose(self, index):
        if (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                return self.aStep.armTrajectory.rArm[index].ee_pose
            else:
                return self.aStep.armTrajectory.lArm[index].ee_pose
        else:
            rospy.logerr('Cannot request trajectory pose on non-trajectory action step.')
            
    def updateVisualization(self):
        menuControl = InteractiveMarkerControl()
        menuControl.interaction_mode = InteractiveMarkerControl.MENU
        menuControl.always_visible = True

        refFrame = self.getReferenceFrame()
        if (refFrame == ArmState.OBJECT):
            frame_id = 'task_object'
        else:
            frame_id = 'base_link'
        pose = self.getPose()
        
        if (self.aStep.type == ActionStep.ARM_TARGET):
            menuControl.markers.append(self.getSphereMarker(self.id, pose, frame_id, 0.08))
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            pointList = []
            for j in range(len(self.aStep.armTrajectory.timing)):
                pointList.append(self.getTrajPose(j).position)
            menuControl.markers.append(Marker(type=Marker.SPHERE_LIST, id=self.id, lifetime=rospy.Duration(2),
                                         scale=Vector3(0.02,0.02,0.02), header=Header(frame_id=frame_id),
                                         color=ColorRGBA(0.8, 0.4, 0.0, 0.8), points=pointList))
            menuControl.markers.append(self.getSphereMarker(self.id+2000, self.getTrajPose(0), frame_id, 0.05))
            menuControl.markers.append(self.getSphereMarker(self.id+3000, self.getTrajPose(len(self.aStep.armTrajectory.timing)-1), frame_id, 0.05))
        else:
            rospy.logerr('Unhandled action step type ' + str(self.aStep.type))

        if (refFrame == ArmState.OBJECT):
            menuControl.markers.append(Marker(type=Marker.ARROW, id=1000+self.id, lifetime=rospy.Duration(2),
                                                     scale=Vector3(0.02,0.03,0.04), header=Header(frame_id='task_object'),
                                                     color=ColorRGBA(1.0, 0.8, 0.2, 0.5), points=[pose.position, Point(0,0,0)]))
        
        self.int_marker = InteractiveMarker()
        self.int_marker.name = self.name
        self.int_marker.header.frame_id = "/base_link"
        self.int_marker.pose = pose
        self.int_marker.scale = 1
        self.int_marker.controls.append(menuControl)

        ActionStepMarker.IMServer.insert(self.int_marker, self.markerFeedback)
        
    def getSphereMarker(self, id, pose, frame_id, radius):
        return Marker(type=Marker.SPHERE, id=id, lifetime=rospy.Duration(2),
                        scale=Vector3(radius,radius,radius), pose=pose, header=Header(frame_id=frame_id),
                        color=ColorRGBA(1.0, 0.5, 0.0, 0.8))        
    
    def markerFeedback(self, feedback):
        print 'feedback.event_type', feedback.event_type

    def changeReferenceFrame(self, feedback):
        currentRefFrame = self.getReferenceFrame()

        self.menuHandler.setCheckState(self.subEntries[currentRefFrame], MenuHandler.UNCHECKED)
        self.menuHandler.setCheckState(feedback.menu_entry_id, MenuHandler.CHECKED )
        newRef = self.subEntries.index(feedback.menu_entry_id)
        
        self.setReferenceFrame(newRef)
        
        rospy.loginfo("Switching reference frame to " + str(self.refNames[newRef]) + " for action " + str(self.name))
        self.updateVisualization()
        self.menuHandler.reApply(ActionStepMarker.IMServer)
        ActionStepMarker.IMServer.applyChanges()

    def getObjectPose(self):
        try:
            t = Arms.tfListener.getLatestCommonTime('base_link', 'task_object')
            (position, orientation) = Arms.tfListener.lookupTransform('base_link', 'task_object', t)
            eePose = Pose()
            eePose.position = Point(position[0], position[1], position[2])
            eePose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
            return eePose
        except:
            rospy.logwarn('Something wrong with transform request.')
            return None








