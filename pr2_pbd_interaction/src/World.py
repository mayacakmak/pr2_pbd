import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('pr2_interactive_object_detection')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('tabletop_collision_map_processing')

# Generic libraries
import sys, os, time, threading
from numpy import *
from numpy.linalg import norm

# ROS libraries
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from tf import TransformListener, TransformBroadcaster
from object_manipulation_msgs.msg import ClusterBoundingBox
from object_manipulation_msgs.srv import FindClusterBoundingBox
from pr2_interactive_object_detection.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

# Local stuff
from Response import *
from pr2_pbd_interaction.msg import *

class WorldObject:
    def __init__(self, pose, index, dimensions, isRecognized):
        self.pose = pose
        self.index = index
        self.isRecognized = isRecognized
        self.dimensions = dimensions
        self.menuHandler = MenuHandler()
        self.intMarker = None
        self.isRemoved = False
        self.menuHandler.insert('Remove from scene', callback=self.remove)
    
    def remove(self, feedback):
        print 'wil remove something'
        self.isRemoved = True
    
    def getName(self):
        if (self.isRecognized):
            return 'object' + str(self.index)
        else:
            return 'thing' + str(self.index)
        
    def decreseIndex(self):
        self.index -= 1

class World:
    "Object recognition and localization related stuff"
    
    tfListener = None

    def __init__(self):
        
        if World.tfListener == None:
            World.tfListener = TransformListener()

        self.lock = threading.Lock()
        self.objects = []
        self.nRecognizedObjects = 0
        self.nUnrecognizedObjects = 0
        self.tfBroadcaster = TransformBroadcaster()
        self.IMServer = InteractiveMarkerServer('world_objects')
        bbServiceName = 'find_cluster_bounding_box'
        rospy.wait_for_service(bbServiceName)
        self.bbService = rospy.ServiceProxy(bbServiceName, FindClusterBoundingBox)
        rospy.Subscriber('interactive_object_recognition_result', GraspableObjectList, self.receieveRecognizedObjectInfo)
        self.objectActionClient = actionlib.SimpleActionClient('object_detection_user_command', UserCommandAction)
        self.objectActionClient.wait_for_server()
        rospy.loginfo('Interactive object detection action server has responded.')
        self.clearAllObjects()
        
    def resetObjects(self):
        for i in range(len(self.objects)):
            self.IMServer.erase(self.objects[i].intMarker.name)
            self.IMServer.applyChanges()
        self.IMServer.clear()
        self.IMServer.applyChanges()
        self.objects = []
        self.nRecognizedObjects = 0
        self.nUnrecognizedObjects = 0
    
    def receieveRecognizedObjectInfo(self, objectList):
        self.lock.acquire()
        rospy.loginfo('Received recognized object list.')
        if (len(objectList.graspable_objects) > 0):
            for i in range(len(objectList.graspable_objects)):
                if (len(objectList.graspable_objects[i].potential_models) > 0):
                    objectPose = None
                    bestConfidence = 0.0
                    for j in range(len(objectList.graspable_objects[i].potential_models)):
                        if (bestConfidence < objectList.graspable_objects[i].potential_models[j].confidence):
                            objectPose = objectList.graspable_objects[i].potential_models[j].pose.pose
                            bestConfidence = objectList.graspable_objects[i].potential_models[j].confidence
                    if (objectPose != None):
                        rospy.logwarn('Adding the recognized object with most confident model.')
                        self.addNewObject(objectPose, Vector3(0.2, 0.2, 0.2), True, objectList.meshes[i])
                else:
                    rospy.logwarn('... but this is not a recognition result, it is probably just segmentation.')
                    bbox = self.bbService(objectList.graspable_objects[i].cluster)
                    clusterPose = bbox.pose.pose
                    if (clusterPose != None):
                        print 'Adding unrecognized object Cluster pose:', World.pose2string(clusterPose)
                        print 'in reference frame', bbox.pose.header.frame_id
                        self.addNewObject(clusterPose, bbox.box_dims, False)
        else:
            rospy.logwarn('... but the list was empty.')
            
        print 'len(self.objects)', len(self.objects)
        self.lock.release()
        
    def getMeshMarker(self, marker, mesh):
        marker.type = Marker.TRIANGLE_LIST
        t = 0
        marker.scale = Vector3(1.0, 1.0, 1.0)
        while (t+2 < len(mesh.triangles)):
            if ((mesh.triangles[t] < len(mesh.vertices)) and (mesh.triangles[t+1] < len(mesh.vertices)) 
                    and (mesh.triangles[t+2] < len(mesh.vertices))):
                marker.points.append(mesh.vertices[mesh.triangles[t]])
                marker.points.append(mesh.vertices[mesh.triangles[t+1]])
                marker.points.append(mesh.vertices[mesh.triangles[t+2]])
                t += 3
            else:
                rospy.logerr('Mesh contains invalid triangle!');
                break
        return marker
            
    def addNewObject(self, pose, dimensions, isRecognized, mesh=None):
        isSameThreshold = 0.02
        toRemove = None
        if (isRecognized):
            # Check if there is already an object
            for i in range(len(self.objects)):
                print 'Distance from previous object',i,':',World.poseDistance(self.objects[i].pose, pose)
                if (World.poseDistance(self.objects[i].pose, pose) < isSameThreshold):
                    if (self.objects[i].isRecognized):
                        rospy.loginfo('** Previously recognized object at the same location, will not add this object.')
                        return False
                    else:
                        rospy.loginfo('** Previously unrecognized object at the same location, will replace it with the recognized object.')
                        toRemove = i
                        break

            if (toRemove != None):
                self.removeObject(toRemove)

            self.objects.append(WorldObject(pose, self.nRecognizedObjects, dimensions, isRecognized))
            self.objects[-1].intMarker = self.getObjectMarker(len(self.objects)-1, mesh)
            self.IMServer.insert(self.objects[-1].intMarker, self.markerFeedback)
            self.IMServer.applyChanges()
            self.objects[-1].menuHandler.apply(self.IMServer, self.objects[-1].intMarker.name)
            self.IMServer.applyChanges()
            self.nRecognizedObjects += 1
            return True
        else:
            for i in range(len(self.objects)):
                print '(unrec)Distance from previous object:', World.poseDistance(self.objects[i].pose, pose)
                if (World.poseDistance(self.objects[i].pose, pose) < isSameThreshold):
                    rospy.loginfo('Previously detected object at the same location, will not add this object.')
                    return False

            self.objects.append(WorldObject(pose, self.nUnrecognizedObjects, dimensions, isRecognized))
            self.objects[-1].intMarker = self.getObjectMarker(len(self.objects)-1)
            self.IMServer.insert(self.objects[-1].intMarker, self.markerFeedback)
            self.IMServer.applyChanges()
            self.objects[-1].menuHandler.apply(self.IMServer, self.objects[-1].intMarker.name)
            self.IMServer.applyChanges()
            self.nUnrecognizedObjects += 1
            return True
    
    def removeObject(self, toRemove):
        obj = self.objects.pop(toRemove)
        rospy.loginfo('Removing object ' + obj.intMarker.name)
        self.IMServer.erase(obj.intMarker.name)
        self.IMServer.applyChanges()
        if (obj.isRecognized):
            for i in range(len(self.objects)):
                if ((self.objects[i].isRecognized) and self.objects[i].index>obj.index):
                    self.objects[i].decreseIndex()
            self.nRecognizedObjects -= 1
        else:
            for i in range(len(self.objects)):
                if ((not self.objects[i].isRecognized) and self.objects[i].index>obj.index):
                    self.objects[i].decreseIndex()
            self.nUnrecognizedObjects -= 1
   
    def getObjectMarker(self, index, mesh=None):
        int_marker = InteractiveMarker()
        int_marker.name = self.objects[index].getName()
        #int_marker.description = self.objects[index].getName()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = self.objects[index].pose
        int_marker.scale = 1
        
        buttonControl = InteractiveMarkerControl()
        buttonControl.interaction_mode = InteractiveMarkerControl.BUTTON
        buttonControl.always_visible = True

        objectMarker = Marker(type=Marker.CUBE, id=index, lifetime=rospy.Duration(2),
                              scale=self.objects[index].dimensions, header=Header(frame_id='base_link'),
                              color=ColorRGBA(0.2, 0.8, 0.0, 0.6), pose = self.objects[index].pose) 

        if (mesh != None):
            objectMarker = self.getMeshMarker(objectMarker, mesh)
        buttonControl.markers.append(objectMarker)

        textPos = Point()
        textPos.x = self.objects[index].pose.position.x
        textPos.y = self.objects[index].pose.position.y
        textPos.z = self.objects[index].pose.position.z + self.objects[index].dimensions.z/2 + 0.06
        buttonControl.markers.append(Marker(type=Marker.TEXT_VIEW_FACING, id=index, scale=Vector3(0,0,0.03),
                                            text=int_marker.name, color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                                            header=Header(frame_id='base_link'), pose=Pose(textPos, Quaternion(0,0,0,1))))
        
        int_marker.controls.append(buttonControl)
        return int_marker

    def getReferenceFrameNameList(self):
        objectNames = ['base_link']
        for i in range(len(self.objects)):
            objectNames.append(self.objects[i].getName())
        return objectNames     
    
    def hasObjects(self):
        return len(self.objects) > 0
    
    @staticmethod
    def getRefFromName(refName):
        if refName == 'base_link':
            return ArmState.ROBOT_BASE
        else:
            return ArmState.OBJECT
        
    @staticmethod
    def convertRefFrame(refFrame, refFrameName, armFrame):
        if (armFrame.refFrame != refFrame):
            if refFrame == ArmState.ROBOT_BASE:
                if (armFrame.refFrame == ArmState.OBJECT):
                    absEEPose = World.transform(armFrame.ee_pose, armFrame.refFrameName, 'base_link')
                    armFrame.ee_pose = absEEPose
                    armFrame.refFrame = ArmState.ROBOT_BASE
                    armFrame.refFrameName = 'base_link'
                else:
                    rospy.logerr('Unhandled reference frame conversion:' + str(armFrame.refFrame) + ' to ' + str(refFrame))
            elif refFrame == ArmState.OBJECT:
                if (armFrame.refFrame == ArmState.ROBOT_BASE):
                    relEEPose = World.transform(armFrame.ee_pose, 'base_link', refFrameName)
                    armFrame.ee_pose = relEEPose
                    armFrame.refFrame = ArmState.OBJECT
                    armFrame.refFrameName = refFrameName
                else:
                    rospy.logerr('Unhandled reference frame conversion:' + str(armFrame.refFrame) + ' to ' + str(refFrame))
        return armFrame
        
    @staticmethod
    def transform(pose, fromFrame, toFrame):
        #rospy.loginfo('Making a transformation from ' + fromFrame + " to " + toFrame)
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
            self.resetObjects()

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
                        while (not self.hasObjects() and waitTime < totalWaitTime):
                            time.sleep(0.1)
                            waitTime += 0.1
                            
                        if (not self.hasObjects()):
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
        
    def clearAllObjects(self):
        self.objectActionClient.send_goal(UserCommandGoal(UserCommandGoal.RESET, False))
        while (self.objectActionClient.get_state() == GoalStatus.ACTIVE or 
               self.objectActionClient.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' + self.objectActionClient.get_goal_status_text())
        if (self.objectActionClient.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('Successfully reset object localization pipeline.')
            self.resetObjects()

    def getNearestObject(self, armPose):
        distances = []
        for i in range(len(self.objects)):
            distances.append(World.poseDistance(self.objects[i].pose, armPose))
        thresholdFar = 0.75
        if (len(distances) > 0):
            if (min(distances) < thresholdFar):
                chosen = distances.index(min(distances))
                return self.objects[chosen].getName()
            else:
                return None
        else:
            return None

    @staticmethod
    def poseDistance(poseA, poseB, onTable=True):
        if poseA == [] or poseB == []:
            return 0.0
        else:
            if (onTable):
                dist = norm(array([poseA.position.x,poseA.position.y]) - 
                            array([poseB.position.x,poseB.position.y]))
            else:
                dist = norm(array([poseA.position.x,poseA.position.y,poseA.position.z]) - 
                            array([poseB.position.x,poseB.position.y,poseB.position.z]))
            if dist < 0.0001:
                dist = 0
            return dist

    def markerFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def update(self):
        # Visualize the detected object
        self.lock.acquire()
        if (self.hasObjects()):
            toRemove = None
            for i in range(len(self.objects)):
                self.publishTFPose(self.objects[i].pose, self.objects[i].getName(), 'base_link')
                if (self.objects[i].isRemoved):
                    toRemove = i
            if toRemove != None:
                self.removeObject(toRemove)
        self.lock.release()
                    
                                   