import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('pr2_interactive_object_detection')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')

# Generic libraries
import sys,os,time

# ROS libraries
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from tf import TransformListener, TransformBroadcaster

# Local stuff
from Response import *
from pr2_pbd_interaction.msg import *
from pr2_interactive_object_detection.msg import *
from geometry_msgs.msg import *

class World:
    "Object recognition and localization related stuff"
    
    tfListener = None

    def __init__(self):
        
        if World.tfListener == None:
            World.tfListener = TransformListener()

        self.tfBroadcaster = TransformBroadcaster()

        self.poses = None
        self.index = 0
        rospy.Subscriber('interactive_object_recognition_result', GraspableObjectList, self.receieveRecognizedObjectInfo)
        
        self.objectActionClient = actionlib.SimpleActionClient('object_detection_user_command', UserCommandAction)
        self.objectActionClient.wait_for_server()
        rospy.loginfo('Interactive object detection action server has responded.')
        
    def receieveRecognizedObjectInfo(self, objectList):
        rospy.loginfo('Received recognized object list.')
        if (len(objectList.graspable_objects) > 0):
            for i in range(len(objectList.graspable_objects)):
                print 'reference_frame_id', objectList.graspable_objects[i].reference_frame_id
                if (len(objectList.graspable_objects[i].potential_models) > 0):
                    objectPoses = [None]*len(objectList.graspable_objects)
                    bestConfidence = 0.0
                    chosenModel = -1
                    for j in range(len(objectList.graspable_objects[i].potential_models)):
                        if (bestConfidence < objectList.graspable_objects[i].potential_models[j].confidence):
                            objectPoses[i] = objectList.graspable_objects[i].potential_models[j].pose.pose
                            bestConfidence = objectList.graspable_objects[i].potential_models[j].confidence
                            chosenModel = j
        
                        print 'Potential model', j, ':'
                        print '--- Model id:', objectList.graspable_objects[i].potential_models[j].model_id
                        print '--- Confidence:', objectList.graspable_objects[i].potential_models[j].confidence
                        print '--- Pose:', World.pose2string(objectPoses[i])
                    
                    print 'Chose model:', j
                    print 'poses', objectPoses
                    self.poses = objectPoses
                    
                    objectIndex = 0
                    while(objectIndex<len(self.poses) and self.poses[objectIndex] == None):
                        objectIndex += 1
                    if (objectIndex<len(self.poses)):
                        self.index = objectIndex
                else:
                    rospy.logwarn('... but this is not a recognition result, it is probably just segmentation.')
        else:
            rospy.logwarn('... but the list was empty.')
            
    
    @staticmethod
    def convertRefFrame(refFrame, armFrame, prevFrame=None):
        if (armFrame.refFrame != refFrame):
            if refFrame == ArmState.NOT_MOVING:
                # Nothing to do, recorded info will be ignored
                armFrame.refFrame = ArmState.NOT_MOVING
            elif (armFrame.refFrame == ArmState.NOT_MOVING):
                armFrame.refFrame = prevFrame.refFrame
                armFrame.ee_pose = prevFrame.ee_pose
                armFrame.joint_pose = prevFrame.joint_pose
            else:
                if refFrame == ArmState.ROBOT_BASE:
                    if (armFrame.refFrame == ArmState.OBJECT):
                        absEEPose = World.transform(armFrame.ee_pose, '/task_object', '/base_link')
                        armFrame.ee_pose = absEEPose
                        armFrame.refFrame = ArmState.ROBOT_BASE
                    else:
                        rospy.logerr('Unhandled reference frame conversion:' + str(armFrame.refFrame) + ' to ' + str(refFrame))
                elif refFrame == ArmState.OBJECT:
                    if (armFrame.refFrame == ArmState.ROBOT_BASE):
                        relEEPose = World.transform(armFrame.ee_pose, '/base_link', '/task_object')
                        armFrame.ee_pose = relEEPose
                        armFrame.refFrame = ArmState.OBJECT
                    else:
                        rospy.logerr('Unhandled reference frame conversion:' + str(armFrame.refFrame) + ' to ' + str(refFrame))
        return armFrame
        
    @staticmethod
    def transform(pose, fromFrame, toFrame):
        objPoseStamped = PoseStamped()
        t = World.tfListener.getLatestCommonTime(fromFrame, toFrame)
        objPoseStamped.header.stamp = t
        objPoseStamped.header.frame_id = fromFrame
        objPoseStamped.pose = pose
        relEEPose = World.tfListener.transformPose(toFrame, objPoseStamped)
        return relEEPose.pose


    @staticmethod
    def pose2string(pose):
        return ('Position: ' + str(pose.position.x) + ", " + str(pose.position.y) + ', ' + str(pose.position.z) + '\n' +
                'Orientation: ' + str(pose.orientation.x) + ", " + str(pose.orientation.y) + ', ' + str(pose.orientation.z) + ', ' + str(pose.orientation.w) + '\n');
    
    def publishTFPose(self, pose, name, parent):
        if (pose != None):
            self.tfBroadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                 (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                 rospy.Time.now(), name, parent)
 
    def updateTaskObjectPose(self, param=None):
        Response.performGazeAction(GazeGoal.LOOK_DOWN)
        while (Response.gazeActionClient.get_state() == GoalStatus.PENDING or 
               Response.gazeActionClient.get_state() == GoalStatus.ACTIVE):
            time.sleep(0.1)
        
        if (Response.gazeActionClient.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('Looking at table now.')
            
            self.objectActionClient.send_goal(UserCommandGoal(UserCommandGoal.RESET, False))
            while (self.objectActionClient.get_state() == GoalStatus.ACTIVE or 
                   self.objectActionClient.get_state() == GoalStatus.PENDING):
                time.sleep(0.1)
            rospy.loginfo('Object recognition has been reset.')
            rospy.loginfo('STATUS: ' + self.objectActionClient.get_goal_status_text())

            if (self.objectActionClient.get_state() == GoalStatus.SUCCEEDED):
                # Do segmentation
                self.objectActionClient.send_goal(UserCommandGoal(UserCommandGoal.SEGMENT, False))
                while (self.objectActionClient.get_state() == GoalStatus.ACTIVE or 
                       self.objectActionClient.get_state() == GoalStatus.PENDING):
                    time.sleep(0.1)
                rospy.loginfo('Table segmentation is complete.')
                rospy.loginfo('STATUS: ' + self.objectActionClient.get_goal_status_text())
    
                if (self.objectActionClient.get_state() == GoalStatus.SUCCEEDED):
                    # Do recognition
                    self.poses = None
                    self.objectActionClient.send_goal(UserCommandGoal(UserCommandGoal.RECOGNIZE, False))
                    while (self.objectActionClient.get_state() == GoalStatus.ACTIVE or 
                           self.objectActionClient.get_state() == GoalStatus.PENDING):
                        time.sleep(0.1)
                    rospy.loginfo('Objects on the table have been recognized.')
                    rospy.loginfo('STATUS: ' + self.objectActionClient.get_goal_status_text())
                    # Record the result

                    if (self.objectActionClient.get_state() == GoalStatus.SUCCEEDED):
                        waitTime = 0
                        totalWaitTime = 5
                        while (self.poses == None and waitTime<totalWaitTime):
                            time.sleep(0.1)
                            waitTime += 0.1
                            
                        if (self.poses == None):
                            rospy.logerr('Timeout waiting for a recognition result.')
                            return False
                        else:
                            rospy.loginfo('Got the object list.')
                            return True
                    else:
                        rospy.logerr('Could not recognize.')
                        return False
                else:
                    rospy.logerr('Could not segment.')
                    return False
            else:
                rospy.logerr('Could not reset recognition.')
                return False
        else:
            rospy.logerr('Could not look down to take table snapshot')
            return False
        
        
    def update(self):
        # Visualize the detected object
        if (self.poses != None):
            # Publish the object pose
            self.publishTFPose(self.poses[self.index], 'task_object', 'base_link')
        
