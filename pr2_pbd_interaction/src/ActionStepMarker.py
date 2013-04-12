# Manifests
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('interactive_markers')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('tf')

# Generic libraries
import time, sys, signal
import numpy
from numpy import *
from numpy.linalg import norm
import os
from geometry_msgs.msg import *

# ROS Libraries
import rospy
import rosbag
import tf
from pr2_pbd_interaction.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from Arms import *

class ActionStepMarker:
    
    IMServer = None

    def __init__(self, id, armIndex, aStep, refFrameObjectList):
        
        if ActionStepMarker.IMServer == None:
            ActionStepMarker.IMServer = InteractiveMarkerServer('programmed_actions')

        self.aStep = aStep
        self.armIndex = armIndex
        self.step = id
        self.id = 2*self.step + self.armIndex
        self.name = 'step' + str(self.step) + 'arm' + str(self.armIndex)
        self.isPoseRequested = False
        self.isDeleteRequested = False
        self.poseControlVisible = False
        self.offset = 0.09

        self.updateReferenceFrameList(refFrameObjectList)
        
        armState, self.isReachable = Arms.solveIK4ArmState(self.armIndex, self.getTarget())
        self.updateMenu()

    def decreaseID(self):
        self.step -= 1
        self.id = 2*self.step + self.armIndex
        self.name = 'step' + str(self.step) + 'arm' + str(self.armIndex)
        self.updateMenu()

    def updateReferenceFrameList(self, refFrameObjectList):
        # There is a new list of objects
        # If the current frames are already assigned to object, we need to figure out the correspondences
        self.refFrameObjectList = refFrameObjectList

        armPose = self.getTarget()
        if (armPose.refFrame == ArmState.OBJECT):
            prevRefObject = armPose.refFrameObject
            newRefObject = self.getMostSimilarObject(prevRefObject, refFrameObjectList)
            armPose.refFrameObject = newRefObject
        
        self.updateRefFrameNames()
        armState, self.isReachable = Arms.solveIK4ArmState(self.armIndex, armPose)
        self.updateMenu()
        
    def getMostSimilarObject(self, refObject, refFrameObjectList):
        bestDist = 10000
        chosenObjIndex = -1
        for i in range(len(refFrameObjectList)):
            dist = World.objectDissimilarity(refFrameObjectList[i], refObject)
            if (dist < bestDist):
                bestDist = dist
                chosenObjIndex = i
        if chosenObjIndex==-1:
            print 'Did not find a similar object..'
            return None
        else:
            print 'Most similar to new object ', chosenObjIndex
            return refFrameObjectList[chosenObjIndex]
            
    def updateRefFrameNames(self):
        self.refNames = ['base_link']
        for i in range(len(self.refFrameObjectList)):
            self.refNames.append(self.refFrameObjectList[i].name)
        
    def destroy(self):
        ActionStepMarker.IMServer.erase(self.name)
        ActionStepMarker.IMServer.applyChanges()

    def updateMenu(self):
        self.menuHandler = MenuHandler()
        frameEntry = self.menuHandler.insert('Reference frame')
        self.subEntries = [None]*len(self.refNames)
        for i in range(len(self.refNames)):
            self.subEntries[i] = self.menuHandler.insert(self.refNames[i], parent=frameEntry, callback=self.changeReferenceFrame)
        self.moveMenuEntry = self.menuHandler.insert('Move here', callback=self.moveToPose)
        self.deleteMenuEntry = self.menuHandler.insert('Delete step', callback=self.deleteStep)
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
                if self.aStep.armTarget.rArm.refFrame == ArmState.ROBOT_BASE:
                    return 'base_link'
                else:
                    return self.aStep.armTarget.rArm.refFrameObject.name
            else:
                if self.aStep.armTarget.lArm.refFrame == ArmState.ROBOT_BASE:
                    return 'base_link'
                else:
                    return self.aStep.armTarget.lArm.refFrameObject.name
                
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                if (self.aStep.armTrajectory.rRefFrame == ArmState.ROBOT_BASE):
                    return 'base_link'
                else:
                    return self.aStep.armTrajectory.rRefFrameOject.name
            else:
                if (self.aStep.armTrajectory.lRefFrame == ArmState.ROBOT_BASE):
                    return 'base_link'
                else:
                    return self.aStep.armTrajectory.lRefFrameObject.name
        else:
            rospy.logerr('Unhandled marker type: ' + str(self.aStep.type))
        
    def setReferenceFrame(self, newRefName):
        newRef = World.getRefFromName(newRefName)
        if (newRef != ArmState.ROBOT_BASE):
            index = self.refNames.index(newRefName)
            newRefObject = self.refFrameObjectList[index-1]
        else:
            newRefObject = Object()
        
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                self.aStep.armTarget.rArm = World.convertRefFrame(self.aStep.armTarget.rArm, newRef, newRefObject)
            else:
                self.aStep.armTarget.lArm = World.convertRefFrame(self.aStep.armTarget.lArm, newRef, newRefObject)
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            for i in range(len(self.aStep.armTrajectory.timing)):
                if self.armIndex == 0:
                    self.aStep.armTrajectory.rArm[i] = World.convertRefFrame(self.aStep.armTrajectory.rArm[i], newRef, newRefObject)
                else:
                    self.aStep.armTrajectory.lArm[i] = World.convertRefFrame(self.aStep.armTrajectory.lArm[i], newRef, newRefObject)
            if self.armIndex == 0:
                self.aStep.armTrajectory.rRefFrameObject = newRefObject
                self.aStep.armTrajectory.rRefFrame = newRef
            else:
                self.aStep.armTrajectory.lRefFrameObject = newRefObject
                self.aStep.armTrajectory.lRefFrame = newRef

    def isHandOpen(self):
            if self.armIndex == 0:
                return (self.aStep.gripperAction.rGripper == GripperState.OPEN)
            else:
                return (self.aStep.gripperAction.lGripper == GripperState.OPEN)

    def setNewPose(self, newPose):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                self.aStep.armTarget.rArm.ee_pose = self.offsetPose(newPose, -1)
            else:
                self.aStep.armTarget.lArm.ee_pose = self.offsetPose(newPose, -1)
            armState, self.isReachable = Arms.solveIK4ArmState(self.armIndex, self.getTarget())
            self.updateVisualization()
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            rospy.logwarn('Modification of whole trajectory segments is not implemented.')

    def getFrameAbsolutePose(self, armState):
        if (armState.refFrame == ArmState.OBJECT):
            armStateCopy = ArmState(armState.refFrame, 
                                    Pose(armState.ee_pose.position, armState.ee_pose.orientation), 
                                    armState.joint_pose[:], armState.refFrameObject)
            World.convertRefFrame(armStateCopy, ArmState.ROBOT_BASE)
            return armStateCopy.ee_pose
        else:
            return armState.ee_pose
            
    def getAbsolutePosition(self, isStart=True):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.offsetPose(self.getFrameAbsolutePose(self.aStep.armTarget.rArm)).position
            else:
                return self.offsetPose(self.getFrameAbsolutePose(self.aStep.armTarget.lArm)).position
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                if isStart:
                    return self.offsetPose(self.getFrameAbsolutePose(self.aStep.armTrajectory.rArm[len(self.aStep.armTrajectory.rArm)-1])).position
                else:
                    return self.offsetPose(self.getFrameAbsolutePose(self.aStep.armTrajectory.rArm[0])).position
            else:
                if isStart:
                    return self.offsetPose(self.getFrameAbsolutePose(self.aStep.armTrajectory.lArm[len(self.aStep.armTrajectory.rArm)-1])).position
                else:
                    return self.offsetPose(self.getFrameAbsolutePose(self.aStep.armTrajectory.lArm[0])).position
               
    def getPose(self):
        t = self.getTarget()
        if (t != None):
            return self.offsetPose(t.ee_pose)
            
    def offsetPose(self, pose, const=1):
            M = self.getMartixFromPose(pose)
            T = tf.transformations.translation_matrix([const*self.offset, 0, 0])
            Mhand = tf.transformations.concatenate_matrices(M, T)
            return self.getPoseFromMartix(Mhand)
                        
    def getTarget(self, trajectoryIndex=None):
        if (self.aStep.type == ActionStep.ARM_TARGET):
            if self.armIndex == 0:
                return self.aStep.armTarget.rArm
            else:
                return self.aStep.armTarget.lArm
        elif (self.aStep.type == ActionStep.ARM_TRAJECTORY):
            if self.armIndex == 0:
                if trajectoryIndex == None:
                    trajectoryIndex = int(len(self.aStep.armTrajectory.rArm)/2)
                return self.aStep.armTrajectory.rArm[trajectoryIndex]
            else:
                if trajectoryIndex == None:
                    trajectoryIndex = int(len(self.aStep.armTrajectory.lArm)/2)
                return self.aStep.armTrajectory.lArm[trajectoryIndex]

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
            #mainMarker = self.getSphereMarker(self.id, pose, frame_id, 0.08)
            #menuControl.markers.append(mainMarker)
            menuControl = self.makeGripperMarker(menuControl, self.isHandOpen())
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
        textPos.z = pose.position.z + 0.1
        menuControl.markers.append(Marker(type=Marker.TEXT_VIEW_FACING, id=self.id, scale=Vector3(0,0,0.03),
                                            text='Step'+str(self.step), color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                                            header=Header(frame_id=frame_id), pose=Pose(textPos, Quaternion(0,0,0,1))))

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

    def deleteStep(self, feedback):
        self.isDeleteRequested = True

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

    def getPoseFromMartix(self, M):
        pos = M[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(M)
        return Pose(Point(pos[0], pos[1], pos[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))

    def getMartixFromPose(self, p):
        T = tf.transformations.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        T[:3, 3] = [p.position.x, p.position.y, p.position.z]
        return T
        
    def getMeshMarker(self):
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False;
        mesh.type = Marker.MESH_RESOURCE;
        mesh.scale.x = 1.0;
        mesh.scale.y = 1.0;
        mesh.scale.z = 1.0;
        if self.isReachable:
            mesh.color = ColorRGBA(1.0, 0.5, 0.0, 0.6) #ColorRGBA(0.8, 0.0, 0.4, 0.8);
        else:
            mesh.color = ColorRGBA(0.5, 0.5, 0.5, 0.6)
        return mesh

    def makeGripperMarker(self, control, isHandOpen=False):
        if isHandOpen:
            angle = 28*numpy.pi/180.0
        else:
            angle=0

        T1 = tf.transformations.euler_matrix(0, 0, angle)
        T1[:3, 3] = [0.07691-self.offset, 0.01, 0]
        T2 = tf.transformations.euler_matrix(0, 0, -angle) 
        T2[:3, 3] = [0.09137, 0.00495, 0]
        T_proximal = T1;
        T_distal = tf.transformations.concatenate_matrices(T1, T2)
        
        mesh1 = self.getMeshMarker()
        mesh1.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
        mesh1.pose.position.x = -self.offset
        mesh1.pose.orientation.w = 1;
        
        mesh2 = self.getMeshMarker()
        mesh2.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
        mesh2.pose = self.getPoseFromMartix(T_proximal)
        
        mesh3 = self.getMeshMarker()
        mesh3.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
        mesh3.pose = self.getPoseFromMartix(T_distal);
        
        q = tf.transformations.quaternion_multiply(tf.transformations.quaternion_from_euler(numpy.pi, 0, 0),tf.transformations.quaternion_from_euler(0, 0, angle))
        T1 = tf.transformations.quaternion_matrix(q)
        T1[:3, 3] = [0.07691-self.offset, -0.01, 0]
        T2 = tf.transformations.euler_matrix(0, 0, -angle) 
        T2[:3, 3] = [0.09137, 0.00495, 0]
        T_proximal = T1;
        T_distal = tf.transformations.concatenate_matrices(T1, T2)
        
        mesh4 = self.getMeshMarker()
        mesh4.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
        mesh4.pose = self.getPoseFromMartix(T_proximal)
        mesh5 = self.getMeshMarker()
        mesh5.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
        mesh5.pose = self.getPoseFromMartix(T_distal);

        control.markers.append( mesh1 );
        control.markers.append( mesh2 );
        control.markers.append( mesh3 );
        control.markers.append( mesh4 );
        control.markers.append( mesh5 );
        
        return control

          
          
          
          
