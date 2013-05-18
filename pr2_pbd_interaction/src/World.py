'''Everything related to perception of the world'''
import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('object_manipulation_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('tabletop_collision_map_processing')
roslib.load_manifest('pr2_interactive_object_detection')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('pr2_social_gaze')

# Generic libraries
import time
import threading
from numpy.linalg import norm
from numpy import array

# ROS libraries
#from actionlib_msgs.msg import
import rospy
import tf
from tf import TransformListener, TransformBroadcaster
from object_manipulation_msgs.msg import GraspableObjectList
from object_manipulation_msgs.srv import FindClusterBoundingBox
from pr2_interactive_object_detection.msg import UserCommandAction, UserCommandGoal
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from actionlib_msgs.msg import GoalStatus
import actionlib

# Local stuff
from pr2_pbd_interaction.msg import Object, ArmState
from pr2_social_gaze.msg import GazeGoal
from Response import Response

class WorldObject:
    '''Class for representing objects'''

    def __init__(self, pose, index, dimensions, is_recognized):
        ''' Initialization of objects'''
        self.index = index
        self.assigned_name = None
        self.is_recognized = is_recognized
        self.object = Object(Object.TABLE_TOP, self.get_name(),
                             pose, dimensions)
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False
        self.menu_handler.insert('Remove from scene', callback=self.remove)

    def remove(self, feedback):
        '''Function for removing object from the world'''
        rospy.loginfo('Will remove object' + self.get_name())
        self.is_removed = True

    def assign_name(self, name):
        '''Function for assigning a different name'''
        self.assigned_name = name
        self.object.name = name

    def get_name(self):
        '''Function to get the object name'''
        if (self.assigned_name == None):
            if (self.is_recognized):
                return 'object' + str(self.index)
            else:
                return 'thing' + str(self.index)
        else:
            return self.assigned_name

    def decrease_index(self):
        '''Function to decrese object index'''
        self.index -= 1


class WorldSurface:
    '''Class for representing surfaces'''

    def __init__(self, pose, dimensions):
        '''Initialization of surface'''
        self.pose = pose
        self.dimensions = dimensions
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False


class World:
    '''Object recognition and localization related stuff'''

    tf_listener = None

    def __init__(self):

        if World.tf_listener == None:
            World.tf_listener = TransformListener()
        self._lock = threading.Lock()
        self.objects = []
        self.surface = None
        self._tf_broadcaster = TransformBroadcaster()
        self._im_server = InteractiveMarkerServer('world_objects')
        bb_service_name = 'find_cluster_bounding_box'
        rospy.wait_for_service(bb_service_name)
        self._bb_service = rospy.ServiceProxy(bb_service_name,
                                            FindClusterBoundingBox)
        rospy.Subscriber('interactive_object_recognition_result',
            GraspableObjectList, self.receieve_object_info)
        self._object_action_client = actionlib.SimpleActionClient(
            'object_detection_user_command', UserCommandAction)
        self._object_action_client.wait_for_server()
        rospy.loginfo('Interactive object detection action ' +
                      'server has responded.')
        self.clear_all_objects()
        # The following is to get the table information
        rospy.Subscriber('tabletop_segmentation_markers',
                         Marker, self.receieve_table_marker)

    def reset_objects(self):
        '''Function that removes all objects'''
        self._lock.acquire()
        for i in range(len(self.objects)):
            self._im_server.erase(self.objects[i].int_marker.name)
            self._im_server.applyChanges()
        if self.surface != None:
            self.remove_surface()
        self._im_server.clear()
        self._im_server.applyChanges()
        self.objects = []
        self._lock.release()

    def receieve_table_marker(self, marker):
        '''Callback function for markers to determine table'''
        if (marker.type == Marker.LINE_STRIP):
            if (len(marker.points) == 6):
                rospy.loginfo('Received a TABLE marker.')
                xmin = marker.points[0].x
                ymin = marker.points[0].y
                xmax = marker.points[2].x
                ymax = marker.points[2].y
                depth = xmax - xmin
                width = ymax - ymin

                pose = Pose(marker.pose.position, marker.pose.orientation)
                pose.position.x = pose.position.x + xmin + depth / 2
                pose.position.y = pose.position.y + ymin + width / 2

                self.surface = WorldSurface(marker.pose,
                                    Vector3(depth, width, 0.01))
                self.surface.int_marker = self._get_surface_marker()
                self._im_server.insert(self.surface.int_marker,
                                     self.marker_feedback_cb)
                self._im_server.applyChanges()

    def receieve_object_info(self, object_list):
        '''Callback function to receive object info'''
        self._lock.acquire()
        rospy.loginfo('Received recognized object list.')
        if (len(object_list.graspable_objects) > 0):
            for i in range(len(object_list.graspable_objects)):
                models = object_list.graspable_objects[i].potential_models
                if (len(models) > 0):
                    object_pose = None
                    best_confidence = 0.0
                    for j in range(len(models)):
                        if (best_confidence < models[j].confidence):
                            object_pose = models[j].pose.pose
                            best_confidence = models[j].confidence
                    if (object_pose != None):
                        rospy.logwarn('Adding the recognized object ' +
                                      'with most confident model.')
                        self._add_new_object(object_pose,
                            Vector3(0.2, 0.2, 0.2), True,
                            object_list.meshes[i])
                else:
                    rospy.logwarn('... this is not a recognition result, ' +
                                  'it is probably just segmentation.')
                    cluster = object_list.graspable_objects[i].cluster
                    bbox = self._bb_service(cluster)
                    cluster_pose = bbox.pose.pose
                    if (cluster_pose != None):
                        rospy.loginfo('Adding unrecognized object with pose:' +
                            World.pose_to_string(cluster_pose) + '\n' +
                            'In ref frame' + str(bbox.pose.header.frame_id))
                        self._add_new_object(cluster_pose, bbox.box_dims,
                                             False)
        else:
            rospy.logwarn('... but the list was empty.')
        self._lock.release()

    @staticmethod
    def _get_mesh_marker(marker, mesh):
        '''Function that generated a marker from a mesh'''
        marker.type = Marker.TRIANGLE_LIST
        index = 0
        marker.scale = Vector3(1.0, 1.0, 1.0)
        while (index + 2 < len(mesh.triangles)):
            if ((mesh.triangles[index] < len(mesh.vertices))
                    and (mesh.triangles[index + 1] < len(mesh.vertices))
                    and (mesh.triangles[index + 2] < len(mesh.vertices))):
                marker.points.append(mesh.vertices[mesh.triangles[index]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 1]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 2]])
                index += 3
            else:
                rospy.logerr('Mesh contains invalid triangle!')
                break
        return marker

    def _add_new_object(self, pose, dimensions, is_recognized, mesh=None):
        '''Function to add new objects'''
        dist_threshold = 0.02
        to_remove = None
        if (is_recognized):
            # Temporary HACK for testing.
            # Will remove all recognition completely if this works.
            return False
            # Check if there is already an object
            for i in range(len(self.objects)):
                distance = World.pose_distance(self.objects[i].object.pose,
                                               pose)
                if (distance < dist_threshold):
                    if (self.objects[i].is_recognized):
                        rospy.loginfo('Previously recognized object at ' +
                            'the same location, will not add this object.')
                        return False
                    else:
                        rospy.loginfo('Previously unrecognized object at ' +
                            'the same location, will replace it with the ' +
                            'recognized object.')
                        to_remove = i
                        break

            if (to_remove != None):
                self.remove_object(to_remove)

            n_objects = len(self.objects)
            self.objects.append(WorldObject(pose, n_objects,
                                            dimensions, is_recognized))
            int_marker = self._get_object_marker(len(self.objects) - 1, mesh)
            self.objects[-1].int_marker = int_marker
            self._im_server.insert(int_marker, self.marker_feedback_cb)
            self._im_server.applyChanges()
            self.objects[-1].menu_handler.apply(self._im_server,
                                               int_marker.name)
            self._im_server.applyChanges()
            return True
        else:
            for i in range(len(self.objects)):
                if (World.pose_distance(self.objects[i].object.pose, pose)
                        < dist_threshold):
                    rospy.loginfo('Previously detected object at the same' +
                                  'location, will not add this object.')
                    return False

            n_objects = len(self.objects)
            self.objects.append(WorldObject(pose, n_objects,
                                            dimensions, is_recognized))
            int_marker = self._get_object_marker(len(self.objects) - 1)
            self.objects[-1].int_marker = int_marker
            self._im_server.insert(int_marker, self.marker_feedback_cb)
            self._im_server.applyChanges()
            self.objects[-1].menu_handler.apply(self._im_server,
                                               int_marker.name)
            self._im_server.applyChanges()
            return True

    def remove_object(self, to_remove):
        '''Function to remove object by index'''
        obj = self.objects.pop(to_remove)
        rospy.loginfo('Removing object ' + obj.int_marker.name)
        self._im_server.erase(obj.int_marker.name)
        self._im_server.applyChanges()
#        if (obj.is_recognized):
#            for i in range(len(self.objects)):
#                if ((self.objects[i].is_recognized)
#                    and self.objects[i].index>obj.index):
#                    self.objects[i].decrease_index()
#            self.n_recognized -= 1
#        else:
#            for i in range(len(self.objects)):
#                if ((not self.objects[i].is_recognized) and
#                    self.objects[i].index>obj.index):
#                    self.objects[i].decrease_index()
#            self.n_unrecognized -= 1

    def remove_surface(self):
        '''Function to request removing surface'''
        rospy.loginfo('Removing surface')
        self._im_server.erase(self.surface.int_marker.name)
        self._im_server.applyChanges()
        self.surface = None

    def _get_object_marker(self, index, mesh=None):
        '''Generate a marker for world objects'''
        int_marker = InteractiveMarker()
        int_marker.name = self.objects[index].get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = self.objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        object_marker = Marker(type=Marker.CUBE, id=index,
                lifetime=rospy.Duration(2),
                scale=self.objects[index].object.dimensions,
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.2, 0.8, 0.0, 0.6),
                pose=self.objects[index].object.pose)

        if (mesh != None):
            object_marker = World._get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        text_pos = Point()
        text_pos.x = self.objects[index].object.pose.position.x
        text_pos.y = self.objects[index].object.pose.position.y
        text_pos.z = (self.objects[index].object.pose.position.z +
                     self.objects[index].object.dimensions.z / 2 + 0.06)
        button_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
                id=index, scale=Vector3(0, 0, 0.03),
                text=int_marker.name, color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                header=Header(frame_id='base_link'),
                pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))
        int_marker.controls.append(button_control)
        return int_marker

    def _get_surface_marker(self):
        ''' Function that generates a surface marker'''
        int_marker = InteractiveMarker()
        int_marker.name = 'surface'
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = self.surface.pose
        int_marker.scale = 1
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        object_marker = Marker(type=Marker.CUBE, id=2000,
                            lifetime=rospy.Duration(2),
                            scale=self.surface.dimensions,
                            header=Header(frame_id='base_link'),
                            color=ColorRGBA(0.8, 0.0, 0.4, 0.4),
                            pose=self.surface.pose)
        button_control.markers.append(object_marker)
        text_pos = Point()
        position = self.surface.pose.position
        dimensions = self.surface.dimensions
        text_pos.x = position.x + dimensions.x / 2 - 0.06
        text_pos.y = position.y - dimensions.y / 2 + 0.06
        text_pos.z = position.z + dimensions.z / 2 + 0.06
        text_marker = Marker(type=Marker.TEXT_VIEW_FACING, id=2001,
                scale=Vector3(0, 0, 0.03), text=int_marker.name,
                color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                header=Header(frame_id='base_link'),
                pose=Pose(text_pos, Quaternion(0, 0, 0, 1)))
        button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)
        return int_marker

    def get_frame_list(self):
        '''Function that returns the list of ref. frames'''
        objects = []
        for i in range(len(self.objects)):
            objects.append(self.objects[i].object)
        return objects

    def has_objects(self):
        '''Function that checks if there are any objects'''
        return len(self.objects) > 0

    @staticmethod
    def object_dissimilarity(obj1, obj2):
        '''Distance between two objects'''
        dims1 = obj1.dimensions
        dims2 = obj2.dimensions
        return norm(array([dims1.x, dims1.y, dims1.z]) -
                    array([dims2.x, dims2.y, dims2.z]))

    @staticmethod
    def get_ref_from_name(ref_name):
        '''Ref. frame type from ref. frame name'''
        if ref_name == 'base_link':
            return ArmState.ROBOT_BASE
        else:
            return ArmState.OBJECT

    @staticmethod
    def convert_ref_frame(arm_frame, ref_frame, ref_frame_obj=Object()):
        '''Transforms an arm frame to a new ref. frame'''
        if ref_frame == ArmState.ROBOT_BASE:
            if (arm_frame.refFrame == ArmState.ROBOT_BASE):
                rospy.logwarn('No reference frame transformations ' +
                              'needed (both absolute).')
            elif (arm_frame.refFrame == ArmState.OBJECT):
                abs_ee_pose = World.transform(arm_frame.ee_pose,
                                arm_frame.refFrameObject.name, 'base_link')
                arm_frame.ee_pose = abs_ee_pose
                arm_frame.refFrame = ArmState.ROBOT_BASE
                arm_frame.refFrameObject = Object()
            else:
                rospy.logerr('Unhandled reference frame conversion:' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        elif ref_frame == ArmState.OBJECT:
            if (arm_frame.refFrame == ArmState.ROBOT_BASE):
                rel_ee_pose = World.transform(arm_frame.ee_pose,
                            'base_link', ref_frame_obj.name)
                arm_frame.ee_pose = rel_ee_pose
                arm_frame.refFrame = ArmState.OBJECT
                arm_frame.refFrameObject = ref_frame_obj
            elif (arm_frame.refFrame == ArmState.OBJECT):
                if (arm_frame.refFrameObject.name == ref_frame_obj.name):
                    rospy.logwarn('No reference frame transformations ' +
                                  'needed (same object).')
                else:
                    rel_ee_pose = World.transform(arm_frame.ee_pose,
                        arm_frame.refFrameObject.name, ref_frame_obj.name)
                    arm_frame.ee_pose = rel_ee_pose
                    arm_frame.refFrame = ArmState.OBJECT
                    arm_frame.refFrameObject = ref_frame_obj
            else:
                rospy.logerr('Unhandled reference frame conversion:' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        return arm_frame

    @staticmethod
    def transform(pose, from_frame, to_frame):
        ''' Function to transform a pose between two ref. frames'''
        pose_stamped = PoseStamped()
        try:
            common_time = World.tf_listener.getLatestCommonTime(from_frame,
                                                                 to_frame)
            pose_stamped.header.stamp = common_time
            pose_stamped.header.frame_id = from_frame
            pose_stamped.pose = pose
            rel_ee_pose = World.tf_listener.transformPose(to_frame,
                                                           pose_stamped)
            return rel_ee_pose.pose
        except tf.Exception, exp1:
            rospy.logerr('TF exception during transform.')
            return pose
        except rospy.ServiceException, exp2:
            rospy.logerr('Exception during transform.')
            return pose

    @staticmethod
    def pose_to_string(pose):
        '''For printing a pose to stdout'''
        return ('Position: ' + str(pose.position.x) + ", " +
                str(pose.position.y) + ', ' + str(pose.position.z) + '\n' +
                'Orientation: ' + str(pose.orientation.x) + ", " +
                str(pose.orientation.y) + ', ' + str(pose.orientation.z) +
                ', ' + str(pose.orientation.w) + '\n')

    def publish_tf_pose(self, pose, name, parent):
        ''' Publishes a TF for object pose'''
        if (pose != None):
            pos = (pose.position.x, pose.position.y, pose.position.z)
            rot = (pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w)
            self._tf_broadcaster.sendTransform(pos, rot,
                                        rospy.Time.now(), name, parent)

    def update_object_pose(self):
        ''' Function to externally update an object pose'''
        Response.performGazeAction(GazeGoal.LOOK_DOWN)
        while (Response.gazeActionClient.get_state() == GoalStatus.PENDING or
               Response.gazeActionClient.get_state() == GoalStatus.ACTIVE):
            time.sleep(0.1)

        if (Response.gazeActionClient.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not look down to take table snapshot')
            return False

        rospy.loginfo('Looking at table now.')
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        self.reset_objects()

        if (self._object_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not reset recognition.')
            return False

        # Do segmentation
        goal = UserCommandGoal(UserCommandGoal.SEGMENT, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Table segmentation is complete.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())

        if (self._object_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not segment.')
            return False

        # Do recognition
        goal = UserCommandGoal(UserCommandGoal.RECOGNIZE, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Objects on the table have been recognized.')
        rospy.loginfo('STATUS: ' + 
                      self._object_action_client.get_goal_status_text())

        # Record the result
        if (self._object_action_client.get_state() == GoalStatus.SUCCEEDED):
            wait_time = 0
            total_wait_time = 5
            while (not self.has_objects() and wait_time < total_wait_time):
                time.sleep(0.1)
                wait_time += 0.1

            if (not self.has_objects()):
                rospy.logerr('Timeout waiting for a recognition result.')
                return False
            else:
                rospy.loginfo('Got the object list.')
                return True
        else:
            rospy.logerr('Could not recognize.')
            return False

    def clear_all_objects(self):
        '''Removes all objects from the world'''
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        if (self._object_action_client.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('Successfully reset object localization pipeline.')
            self.reset_objects()

    def get_nearest_object(self, arm_pose):
        '''Gives a pointed to the nearest object'''
        distances = []
        for i in range(len(self.objects)):
            dist = World.pose_distance(self.objects[i].object.pose,
                                                            arm_pose)
            distances.append(dist)
        dist_threshold = 0.4
        if (len(distances) > 0):
            if (min(distances) < dist_threshold):
                chosen = distances.index(min(distances))
                return self.objects[chosen].object
            else:
                return None
        else:
            return None

    @staticmethod
    def pose_distance(pose1, pose2, is_on_table=True):
        '''Distance between two world poses'''
        if pose1 == [] or pose2 == []:
            return 0.0
        else:
            if (is_on_table):
                arr1 = array([pose1.position.x, pose1.position.y])
                arr2 = array([pose2.position.x, pose2.position.y])
            else:
                arr1 = array([pose1.position.x,
                              pose1.position.y, pose1.position.z])
                arr2 = array([pose2.position.x,
                              pose2.position.y, pose2.position.z])
            dist = norm(arr1 - arr2)
            if dist < 0.0001:
                dist = 0
            return dist

    def marker_feedback_cb(self, feedback):
        '''Callback for when feedback from a marker is received'''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(self.objects)))
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def update(self):
        '''Update function called in a loop'''
        # Visualize the detected object
        is_world_changed = False
        self._lock.acquire()
        if (self.has_objects()):
            to_remove = None
            for i in range(len(self.objects)):
                self.publish_tf_pose(self.objects[i].object.pose,
                    self.objects[i].get_name(), 'base_link')
                if (self.objects[i].is_removed):
                    to_remove = i
            if to_remove != None:
                self.remove_object(to_remove)
                is_world_changed = True
            if (self.surface != None and self.surface.is_removed):
                self.remove_surface()
        self._lock.release()
        return is_world_changed
