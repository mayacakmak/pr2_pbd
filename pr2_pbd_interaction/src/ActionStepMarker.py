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

    def __init__(self, id, armIndex, aStep, refFrameList):
        
        if ActionStepMarker.IMServer == None:
            ActionStepMarker.IMServer = InteractiveMarkerServer('programmed_actions')

        self.refNames = refFrameList
        self.aStep = aStep
        self.armIndex = armIndex
        self.step = id
        self.id = 2*id + self.armIndex
        self.name = 'step' + str(id) + 'arm' + str(self.armIndex)
        self.isPoseRequested = False
        self.poseControlVisible = False
        
        self.updateMenu()

    def updateReferenceFrameList(self, refFrameList):
        self.refNames = refFrameList
        self.updateMenu()
            
    def updateMenu(self):
        self.menuHandler = MenuHandler()
        frameEntry = self.menuHandler.insert('Reference frame')
        self.subEntries = [None]*len(self.refNames)
        for i in range(len(self.refNames)):
            self.subEntries[i] = self.menuHandler.insert(self.refNames[i], parent=frameEntry, callback=self.changeReferenceFrame)

        self.moveMenuEntry = self.menuHandler.insert('Move here', callback=self.moveToPose)

        for i in range(len(self.refNames)):
            self.menuHandler.setCheckState(self.subEntries[i], MenuHandler.UNCHECKED)
        self.menuHandler.setCheckState(self.getMenuIDFromName(self.getReferenceFrameName()), MenuHandler.CHECKED)

        self.updateVisualizationCore()

        self.menuHandler.apply(ActionStepMarker.IMServer, self.name)
        ActionStepMarker.IMServer.applyChanges()

    def getMenuIDFromName(self, refName):
        index = self.refNames.index(refName)
        return self.subEntries[index]
        
    def getNameFromMenuID(self, id):
        index = self.subEntries.index(id)
        return self.refNames[index]

    def getReferenceFrameName(self):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.aStep.armTarget.rArm.refFrameName
            else:
                return self.aStep.armTarget.lArm.refFrameName
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                return self.aStep.armTrajectory.rRefFrameName
            else:
                return self.aStep.armTrajectory.lRefFrameName
        else:
            rospy.logerr('Unhandled marker type: ' + str(self.aStep.type))
        
    def setReferenceFrame(self, newRefName):
        newRef = World.getRefFromName(newRefName) 
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                self.aStep.armTarget.rArm = World.convertRefFrame(newRef, newRefName, self.aStep.armTarget.rArm)
            else:
                self.aStep.armTarget.lArm = World.convertRefFrame(newRef, newRefName, self.aStep.armTarget.lArm)
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            for i in range(len(self.aStep.armTrajectory.timing)):
                if self.armIndex == 0:
                    self.aStep.armTrajectory.rArm[i] = World.convertRefFrame(newRef, newRefName, self.aStep.armTrajectory.rArm[i])
                else:
                    self.aStep.armTrajectory.lArm[i] = World.convertRefFrame(newRef, newRefName, self.aStep.armTrajectory.lArm[i])
            if self.armIndex == 0:
                self.aStep.armTrajectory.rRefFrameName = newRefName
                self.aStep.armTrajectory.rRefFrame = newRef
            else:
                self.aStep.armTrajectory.lRefFrameName = newRefName
                self.aStep.armTrajectory.lRefFrame = newRef

    def setNewPose(self, newPose):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                self.aStep.armTarget.rArm.ee_pose = newPose
            else:
                self.aStep.armTarget.lArm.ee_pose = newPose
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            rospy.logwarn('Modification of whole trajectory segments is not implemented.')

    def getFrameAbsolutePosition(self, armState):
        if (armState.refFrame == ArmState.OBJECT):
            armStateCopy = ArmState(armState.refFrame, 
                                    Pose(armState.ee_pose.position, armState.ee_pose.orientation), 
                                    armState.joint_pose[:], armState.refFrameName)
            World.convertRefFrame(ArmState.ROBOT_BASE, 'base_link', armStateCopy)
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
        t = self.getTarget()
        if (t != None):
            return t.ee_pose
                        
    def getTarget(self):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.aStep.armTarget.rArm
            else:
                return self.aStep.armTarget.lArm
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                return self.aStep.armTrajectory.rArm[int(len(self.aStep.armTrajectory.rArm)/2)]
            else:
                return self.aStep.armTrajectory.lArm[int(len(self.aStep.armTrajectory.lArm)/2)]

    def getTrajPose(self, index):
        if (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                return self.aStep.armTrajectory.rArm[index].ee_pose
            else:
                return self.aStep.armTrajectory.lArm[index].ee_pose
        else:
            rospy.logerr('Cannot request trajectory pose on non-trajectory action step.')
            
    def updateVisualizationCore(self):
        menuControl = InteractiveMarkerControl()
        menuControl.interaction_mode = InteractiveMarkerControl.BUTTON
        menuControl.always_visible = True
        frame_id = self.getReferenceFrameName()
        pose = self.getPose()
        
        if (self.aStep.type == ActionStep.ARM_TARGET):
            mainMarker = self.getSphereMarker(self.id, pose, frame_id, 0.08)
            menuControl.markers.append(mainMarker)
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            pointList = []
            for j in range(len(self.aStep.armTrajectory.timing)):
                pointList.append(self.getTrajPose(j).position)

            mainMarker = Marker(type=Marker.SPHERE_LIST, id=self.id, lifetime=rospy.Duration(2),
                                         scale=Vector3(0.02,0.02,0.02), header=Header(frame_id=frame_id),
                                         color=ColorRGBA(0.8, 0.4, 0.0, 0.8), points=pointList)                
            menuControl.markers.append(mainMarker)
            menuControl.markers.append(self.getSphereMarker(self.id+2000, self.getTrajPose(0), frame_id, 0.05))
            menuControl.markers.append(self.getSphereMarker(self.id+3000, self.getTrajPose(len(self.aStep.armTrajectory.timing)-1), frame_id, 0.05))
        else:
            rospy.logerr('Non-handled action step type ' + str(self.aStep.type))

        refFrame = World.getRefFromName(frame_id)
        if (refFrame == ArmState.OBJECT):
            menuControl.markers.append(Marker(type=Marker.ARROW, id=1000+self.id, lifetime=rospy.Duration(2),
                                                     scale=Vector3(0.02,0.03,0.04), header=Header(frame_id=frame_id),
                                                     color=ColorRGBA(1.0, 0.8, 0.2, 0.5), points=[pose.position, Point(0,0,0)]))
        self.int_marker = InteractiveMarker()
        self.int_marker.name = self.name
        self.int_marker.header.frame_id = frame_id
        self.int_marker.pose = pose
        self.int_marker.scale = 0.2
        self.make6DofMarker(self.int_marker, False)
        
        textPos = Point()
        textPos.x = pose.position.x
        textPos.y = pose.position.y
        textPos.z = pose.position.z + 0.06
        menuControl.markers.append(Marker(type=Marker.TEXT_VIEW_FACING, id=self.id, scale=Vector3(0,0,0.03),
                                            text='Step'+str(self.step), color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                                            header=Header(frame_id='base_link'), pose=Pose(textPos, Quaternion(0,0,0,1))))

        self.int_marker.controls.append(menuControl)
        ActionStepMarker.IMServer.insert(self.int_marker, self.markerFeedback)
        
    def getSphereMarker(self, id, pose, frame_id, radius):
        return Marker(type=Marker.SPHERE, id=id, lifetime=rospy.Duration(2),
                        scale=Vector3(radius,radius,radius), pose=pose, header=Header(frame_id=frame_id),
                        color=ColorRGBA(1.0, 0.5, 0.0, 0.8))  
    
    def markerFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.setNewPose(feedback.pose)
            self.updateVisualization()
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller
            rospy.loginfo('Changing visibility of the pose controls.')
            if (self.poseControlVisible):
                self.poseControlVisible = False
            else:
                self.poseControlVisible = True
            self.updateVisualization()
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def moveToPose(self, feedback):
        self.isPoseRequested = True

    def poseReached(self):
        self.isPoseRequested = False

    def changeReferenceFrame(self, feedback):
        self.menuHandler.setCheckState(self.getMenuIDFromName(self.getReferenceFrameName()), MenuHandler.UNCHECKED)
        self.menuHandler.setCheckState(feedback.menu_entry_id, MenuHandler.CHECKED)
        newRef = self.getNameFromMenuID(feedback.menu_entry_id)
        self.setReferenceFrame(newRef)
        rospy.loginfo("Switching reference frame to " + newRef + " for action step " + str(self.name))
        self.menuHandler.reApply(ActionStepMarker.IMServer)
        ActionStepMarker.IMServer.applyChanges()
        self.updateVisualization()
    
    def updateVisualization(self):
        self.updateVisualizationCore()
        self.menuHandler.reApply(ActionStepMarker.IMServer)
        ActionStepMarker.IMServer.applyChanges()
    
    def make6DofMarker(self, int_marker, fixed ):
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.always_visible = False
        if (self.poseControlVisible):
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.always_visible = False
        if (self.poseControlVisible):
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.always_visible = False
        if (self.poseControlVisible):
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.always_visible = False
        if (self.poseControlVisible):
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.always_visible = False
        if (self.poseControlVisible):
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.always_visible = False
        if (self.poseControlVisible):
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

