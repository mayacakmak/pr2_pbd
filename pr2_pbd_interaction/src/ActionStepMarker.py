'''Stuff related to a single marker for steps of an action'''
import roslib
roslib.load_manifest('pr2_pbd_interaction')

import numpy
import rospy
import tf
from pr2_pbd_interaction.msg import ActionStep, ArmState, Object, GripperState
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from Arms import Arms
from World import World


class ActionStepMarker:
    ''' Marker for visualizing the steps of an action'''

    _im_server = None
    _offset = 0.09
    _ref_object_list = None
    _ref_names = None
    _marker_click_cb = None

    def __init__(self, step_number, arm_index, action_step,
                 marker_click_cb):

        if ActionStepMarker._im_server == None:
            im_server = InteractiveMarkerServer('programmed_actions')
            ActionStepMarker._im_server = im_server

        self.action_step = action_step
        self.arm_index = arm_index
        self.step_number = step_number
        self.is_requested = False
        self.is_deleted = False
        self.is_control_visible = False
        self.is_edited = False
        self.has_object = False

        self._sub_entries = None
        self._menu_handler = None
        ActionStepMarker._marker_click_cb = marker_click_cb

    def _is_reachable(self):
        '''Checks if there is an IK solution for action step'''
        dummy, is_reachable = Arms.solve_ik_for_arm(self.arm_index,
                                                    self.get_target())
        rospy.loginfo('Reachability of pose in ActionStepMarker : ' +
            str(is_reachable))
        return is_reachable

    def get_uid(self):
        '''Returns a unique id of the marker'''
        return (2 * self.step_number + self.arm_index)

    def _get_name(self):
        '''Generates the unique name for the marker'''
        return ('step' + str(self.step_number)
                                        + 'arm' + str(self.arm_index))

    def decrease_id(self):
        '''Reduces the index of the marker'''
        self.step_number -= 1
        self._update_menu()

    def update_ref_frames(self, ref_frame_list):
        '''Updates and re-assigns coordinate frames when the world changes'''
        # There is a new list of objects
        # If the current frames are already assigned to object,
        # we need to figure out the correspondences
        ActionStepMarker._ref_object_list = ref_frame_list

        arm_pose = self.get_target()

        if (arm_pose.refFrame == ArmState.OBJECT):
            prev_ref_obj = arm_pose.refFrameObject
            new_ref_obj = World.get_most_similar_obj(prev_ref_obj,
                                                    ref_frame_list)
            self.has_object = False
            if (new_ref_obj != None):
                self.has_object = True
                arm_pose.refFrameObject = new_ref_obj

        ActionStepMarker._ref_names = ['base_link']
        for i in range(len(ActionStepMarker._ref_object_list)):
            ActionStepMarker._ref_names.append(
                            ActionStepMarker._ref_object_list[i].name)

        self._update_menu()

    def destroy(self):
        '''Removes marker from the world'''
        ActionStepMarker._im_server.erase(self._get_name())
        ActionStepMarker._im_server.applyChanges()

    def _update_menu(self):
        '''Recreates the menu when something has changed'''
        self._menu_handler = MenuHandler()
        frame_entry = self._menu_handler.insert('Reference frame')
        self._sub_entries = [None] * len(ActionStepMarker._ref_names)
        for i in range(len(ActionStepMarker._ref_names)):
            self._sub_entries[i] = self._menu_handler.insert(
                            ActionStepMarker._ref_names[i], parent=frame_entry,
                            callback=self.change_ref_cb)
        self._menu_handler.insert('Move arm here', callback=self.move_to_cb)
        self._menu_handler.insert('Move to current arm pose',
                            callback=self.move_pose_to_cb)
        self._menu_handler.insert('Delete', callback=self.delete_step_cb)
        for i in range(len(ActionStepMarker._ref_names)):
            self._menu_handler.setCheckState(self._sub_entries[i],
                                            MenuHandler.UNCHECKED)

        menu_id = self._get_menu_id(self._get_ref_name())
        if menu_id == None:
            self.has_object = False
        else:
            self._menu_handler.setCheckState(menu_id,
                            MenuHandler.CHECKED)
        self._update_viz_core()
        self._menu_handler.apply(ActionStepMarker._im_server, self._get_name())
        ActionStepMarker._im_server.applyChanges()

    def _get_menu_id(self, ref_name):
        '''Returns the unique menu id from its name
        None if the object is not found'''
        if ref_name in ActionStepMarker._ref_names:
            index = ActionStepMarker._ref_names.index(ref_name)
            return self._sub_entries[index]
        else:
            return None

    def _get_menu_name(self, menu_id):
        '''Returns the menu name from its unique menu id'''
        index = self._sub_entries.index(menu_id)
        return ActionStepMarker._ref_names[index]

    def _get_ref_name(self):
        '''Returns the name string for the reference
        frame object of the action step'''

        ref_name = None
        if (self.action_step.type == ActionStep.ARM_TARGET):
            if self.arm_index == 0:
                ref_frame = self.action_step.armTarget.rArm.refFrame
                ref_name = self.action_step.armTarget.rArm.refFrameObject.name
            else:
                ref_frame = self.action_step.armTarget.lArm.refFrame
                ref_name = self.action_step.armTarget.lArm.refFrameObject.name

        elif (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            if self.arm_index == 0:
                ref_frame = self.action_step.armTrajectory.rRefFrame
                ref_name = self.action_step.armTrajectory.rRefFrameOject.name
            else:
                ref_frame = self.action_step.armTrajectory.lRefFrame
                ref_name = self.action_step.armTrajectory.lRefFrameOject.name
        else:
            rospy.logerr('Unhandled marker type: ' +
                                        str(self.action_step.type))

        if (ref_frame == ArmState.ROBOT_BASE):
            ref_name = 'base_link'

        return ref_name

    def _set_ref(self, new_ref_name):
        '''Changes the reference frame of the action step'''
        new_ref = World.get_ref_from_name(new_ref_name)
        if (new_ref != ArmState.ROBOT_BASE):
            index = ActionStepMarker._ref_names.index(new_ref_name)
            new_ref_obj = ActionStepMarker._ref_object_list[index - 1]
        else:
            new_ref_obj = Object()

        if (self.action_step.type == ActionStep.ARM_TARGET):
            if self.arm_index == 0:
                self.action_step.armTarget.rArm = World.convert_ref_frame(
                        self.action_step.armTarget.rArm, new_ref, new_ref_obj)
            else:
                self.action_step.armTarget.lArm = World.convert_ref_frame(
                        self.action_step.armTarget.lArm, new_ref, new_ref_obj)
        elif (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            for i in range(len(self.action_step.armTrajectory.timing)):
                if self.arm_index == 0:
                    arm_old = self.action_step.armTrajectory.rArm[i]
                    arm_new = World.convert_ref_frame(arm_old,
                                                      new_ref, new_ref_obj)
                    self.action_step.armTrajectory.rArm[i] = arm_new
                else:
                    arm_old = self.action_step.armTrajectory.lArm[i]
                    arm_new = World.convert_ref_frame(arm_old,
                                                      new_ref, new_ref_obj)
                    self.action_step.armTrajectory.lArm[i] = arm_new
            if self.arm_index == 0:
                self.action_step.armTrajectory.rRefFrameObject = new_ref_obj
                self.action_step.armTrajectory.rRefFrame = new_ref
            else:
                self.action_step.armTrajectory.lRefFrameObject = new_ref_obj
                self.action_step.armTrajectory.lRefFrame = new_ref

    def _is_hand_open(self):
        '''Checks if the gripper is open for the action step'''
        if self.arm_index == 0:
            gripper_state = self.action_step.gripperAction.rGripper
        else:
            gripper_state = self.action_step.gripperAction.lGripper
        return (gripper_state == GripperState.OPEN)

    def set_new_pose(self, new_pose):
        '''Changes the pose of the action step'''
        if (self.action_step.type == ActionStep.ARM_TARGET):
            if self.arm_index == 0:
                pose = ActionStepMarker._offset_pose(new_pose, -1)
                self.action_step.armTarget.rArm.ee_pose = pose
            else:
                pose = ActionStepMarker._offset_pose(new_pose, -1)
                self.action_step.armTarget.lArm.ee_pose = pose
            self.update_viz()
        elif (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            rospy.logwarn('Modification of whole trajectory ' +
                          'segments is not implemented.')

    def get_absolute_position(self, is_start=True):
        '''Returns the absolute position of the action step'''
        pose = self.get_absolute_pose(is_start)
        return pose.position

    def get_absolute_pose(self, is_start=True):
        '''Returns the absolute pose of the action step'''
        if (self.action_step.type == ActionStep.ARM_TARGET):
            if self.arm_index == 0:
                arm_pose = self.action_step.armTarget.rArm
            else:
                arm_pose = self.action_step.armTarget.lArm

        elif (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            if self.arm_index == 0:
                if is_start:
                    index = len(self.action_step.armTrajectory.rArm) - 1
                    arm_pose = self.action_step.armTrajectory.rArm[index]
                else:
                    arm_pose = self.action_step.armTrajectory.rArm[0]
            else:
                if is_start:
                    index = len(self.action_step.armTrajectory.lArm) - 1
                    arm_pose = self.action_step.armTrajectory.lArm[index]
                else:
                    arm_pose = self.action_step.armTrajectory.lArm[0]

        #if (arm_pose.refFrame == ArmState.OBJECT and
        #    World.has_object(arm_pose.refFrameObject.name)):
        #    return ActionStepMarker._offset_pose(arm_pose.ee_pose)
        #else:
        world_pose = World.get_absolute_pose(arm_pose)
        return ActionStepMarker._offset_pose(world_pose)

    def get_pose(self):
        '''Returns the pose of the action step'''
        target = self.get_target()
        if (target != None):
            return ActionStepMarker._offset_pose(target.ee_pose)

    @staticmethod
    def _offset_pose(pose, constant=1):
        '''Offsets the world pose for visualization'''
        transform = World.get_matrix_from_pose(pose)
        offset_array = [constant * ActionStepMarker._offset, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(transform,
                                                        offset_transform)
        return World.get_pose_from_transform(hand_transform)

    def set_target(self, target):
        '''Sets the new pose for the action step'''
        if (self.action_step.type == ActionStep.ARM_TARGET):
            if self.arm_index == 0:
                self.action_step.armTarget.rArm = target
            else:
                self.action_step.armTarget.lArm = target
            self.has_object = True
            self._update_menu()
        self.is_edited = False

    def get_target(self, traj_index=None):
        '''Returns the pose for the action step'''
        if (self.action_step.type == ActionStep.ARM_TARGET):
            if self.arm_index == 0:
                return self.action_step.armTarget.rArm
            else:
                return self.action_step.armTarget.lArm
        elif (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            if self.arm_index == 0:
                if traj_index == None:
                    traj = self.action_step.armTrajectory.rArm
                    traj_index = int(len(traj) / 2)
                return self.action_step.armTrajectory.rArm[traj_index]
            else:
                if traj_index == None:
                    traj = self.action_step.armTrajectory.lArm
                    traj_index = int(len(traj) / 2)
                return self.action_step.armTrajectory.lArm[traj_index]

    def _get_traj_pose(self, index):
        '''Returns a single pose for trajectory'''
        if (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            if self.arm_index == 0:
                target = self.action_step.armTrajectory.rArm[index]
            else:
                target = self.action_step.armTrajectory.lArm[index]
            return target.ee_pose
        else:
            rospy.logerr('Cannot request trajectory pose ' +
                         'on non-trajectory action step.')

    def _update_viz_core(self):
        '''Updates visualization after a change'''
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = self._get_ref_name()
        pose = self.get_pose()

        if (self.action_step.type == ActionStep.ARM_TARGET):
            menu_control = self._make_gripper_marker(menu_control,
                                                  self._is_hand_open())
        elif (self.action_step.type == ActionStep.ARM_TRAJECTORY):
            point_list = []
            for j in range(len(self.action_step.armTrajectory.timing)):
                point_list.append(self._get_traj_pose(j).position)

            main_marker = Marker(type=Marker.SPHERE_LIST, id=self.get_uid(),
                                lifetime=rospy.Duration(2),
                                scale=Vector3(0.02, 0.02, 0.02),
                                header=Header(frame_id=frame_id),
                                color=ColorRGBA(0.8, 0.4, 0.0, 0.8),
                                points=point_list)
            menu_control.markers.append(main_marker)
            menu_control.markers.append(ActionStepMarker.make_sphere_marker(
                                self.get_uid() + 2000,
                                self._get_traj_pose(0), frame_id, 0.05))
            last_index = len(self.action_step.armTrajectory.timing) - 1
            menu_control.markers.append(ActionStepMarker.make_sphere_marker(
                self.get_uid() + 3000, self._get_traj_pose(last_index),
                frame_id, 0.05))
        else:
            rospy.logerr('Non-handled action step type '
                         + str(self.action_step.type))

        ref_frame = World.get_ref_from_name(frame_id)
        if (ref_frame == ArmState.OBJECT):
            menu_control.markers.append(Marker(type=Marker.ARROW,
                        id=(1000 + self.get_uid()),
                        lifetime=rospy.Duration(2),
                        scale=Vector3(0.02, 0.03, 0.04),
                        header=Header(frame_id=frame_id),
                        color=ColorRGBA(1.0, 0.8, 0.2, 0.5),
                        points=[pose.position, Point(0, 0, 0)]))

        text_pos = Point()
        text_pos.x = pose.position.x
        text_pos.y = pose.position.y
        text_pos.z = pose.position.z + 0.1
        menu_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
                        id=self.get_uid(), scale=Vector3(0, 0, 0.03),
                        text='Step' + str(self.step_number),
                        color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                        header=Header(frame_id=frame_id),
                        pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))

        int_marker = InteractiveMarker()
        int_marker.name = self._get_name()
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose
        int_marker.scale = 0.2
        self._add_6dof_marker(int_marker, True)

        int_marker.controls.append(menu_control)
        ActionStepMarker._im_server.insert(int_marker,
                                           self.marker_feedback_cb)

    @staticmethod
    def make_sphere_marker(uid, pose, frame_id, radius):
        '''Creates a sphere marker'''
        return Marker(type=Marker.SPHERE, id=uid, lifetime=rospy.Duration(2),
                        scale=Vector3(radius, radius, radius),
                        pose=pose, header=Header(frame_id=frame_id),
                        color=ColorRGBA(1.0, 0.5, 0.0, 0.8))

    def marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker'''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.set_new_pose(feedback.pose)
            self.update_viz()
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller
            rospy.loginfo('Changing visibility of the pose controls.')
            if (self.is_control_visible):
                self.is_control_visible = False
            else:
                self.is_control_visible = True
            ActionStepMarker._marker_click_cb(self.get_uid(),
                                              self.is_control_visible)
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def delete_step_cb(self, dummy):
        '''Callback for when delete is requested'''
        self.is_deleted = True

    def move_to_cb(self, dummy):
        '''Callback for when moving to a pose is requested'''
        self.is_requested = True

    def move_pose_to_cb(self, dummy):
        '''Callback for when a pose change to current is requested'''
        self.is_edited = True

    def pose_reached(self):
        '''Update when a requested pose is reached'''
        self.is_requested = False

    def change_ref_cb(self, feedback):
        '''Callback for when a reference frame change is requested'''
        self._menu_handler.setCheckState(
                    self._get_menu_id(self._get_ref_name()),
                    MenuHandler.UNCHECKED)
        self._menu_handler.setCheckState(feedback.menu_entry_id,
                    MenuHandler.CHECKED)
        new_ref = self._get_menu_name(feedback.menu_entry_id)
        self._set_ref(new_ref)
        rospy.loginfo('Switching reference frame to '
                      + new_ref + ' for action step ' + self._get_name())
        self._menu_handler.reApply(ActionStepMarker._im_server)
        ActionStepMarker._im_server.applyChanges()
        self.update_viz()

    def update_viz(self):
        '''Updates visualization fully'''
        self._update_viz_core()
        self._menu_handler.reApply(ActionStepMarker._im_server)
        ActionStepMarker._im_server.applyChanges()

    def _add_6dof_marker(self, int_marker, is_fixed):
        '''Adds a 6 DoF control marker to the interactive marker'''
        control = self._make_6dof_control('rotate_x',
                        Quaternion(1, 0, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_x',
                        Quaternion(1, 0, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('rotate_z',
                        Quaternion(0, 1, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_z',
                        Quaternion(0, 1, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('rotate_y',
                        Quaternion(0, 0, 1, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_y',
                        Quaternion(0, 0, 1, 1), True, is_fixed)
        int_marker.controls.append(control)

    def _make_6dof_control(self, name, orientation, is_move, is_fixed):
        '''Creates one component of the 6dof controller'''
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.always_visible = False
        if (self.is_control_visible):
            if is_move:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if is_fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        return control

    def _make_mesh_marker(self):
        '''Creates a mesh marker'''
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False
        mesh.type = Marker.MESH_RESOURCE
        mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0
        if self._is_reachable():
            mesh.color = ColorRGBA(1.0, 0.5, 0.0, 0.6)
        else:
            mesh.color = ColorRGBA(0.5, 0.5, 0.5, 0.6)
        return mesh

    def _make_gripper_marker(self, control, is_hand_open=False):
        '''Makes a gripper marker'''
        if is_hand_open:
            angle = 28 * numpy.pi / 180.0
        else:
            angle = 0

        transform1 = tf.transformations.euler_matrix(0, 0, angle)
        transform1[:3, 3] = [0.07691 - ActionStepMarker._offset, 0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(transform1,
                                                           transform2)

        mesh1 = self._make_mesh_marker()
        mesh1.mesh_resource = ('package://pr2_description/meshes/' +
                                'gripper_v0/gripper_palm.dae')
        mesh1.pose.position.x = -ActionStepMarker._offset
        mesh1.pose.orientation.w = 1

        mesh2 = self._make_mesh_marker()
        mesh2.mesh_resource = ('package://pr2_description/meshes/' +
                               'gripper_v0/l_finger.dae')
        mesh2.pose = World.get_pose_from_transform(t_proximal)

        mesh3 = self._make_mesh_marker()
        mesh3.mesh_resource = ('package://pr2_description/meshes/' +
                               'gripper_v0/l_finger_tip.dae')
        mesh3.pose = World.get_pose_from_transform(t_distal)

        quat = tf.transformations.quaternion_multiply(
                    tf.transformations.quaternion_from_euler(numpy.pi, 0, 0),
                    tf.transformations.quaternion_from_euler(0, 0, angle))
        transform1 = tf.transformations.quaternion_matrix(quat)
        transform1[:3, 3] = [0.07691 - ActionStepMarker._offset, -0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(transform1,
                                                           transform2)

        mesh4 = self._make_mesh_marker()
        mesh4.mesh_resource = ('package://pr2_description/meshes/' +
                               'gripper_v0/l_finger.dae')
        mesh4.pose = World.get_pose_from_transform(t_proximal)
        mesh5 = self._make_mesh_marker()
        mesh5.mesh_resource = ('package://pr2_description/meshes/' +
                               'gripper_v0/l_finger_tip.dae')
        mesh5.pose = World.get_pose_from_transform(t_distal)

        control.markers.append(mesh1)
        control.markers.append(mesh2)
        control.markers.append(mesh3)
        control.markers.append(mesh4)
        control.markers.append(mesh5)

        return control
