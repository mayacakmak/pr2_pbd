''' Interface for controlling one arm '''
import roslib
roslib.load_manifest('pr2_pbd_interaction')

import threading
import rospy
import tf
from numpy import array, sign, pi, dot, subtract, diff, divide
from numpy import *
from numpy.linalg import norm
from arm_navigation_msgs.srv import FilterJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient
from pr2_mechanism_msgs.srv import SwitchController
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandAction
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal
from sensor_msgs.msg import JointState
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK
from kinematics_msgs.srv import GetPositionIKRequest
from kinematics_msgs.srv import GetPositionFKRequest, GetPositionFK
from geometry_msgs.msg import Quaternion, Point, Pose
from pr2_pbd_interaction.msg import GripperState, ArmMode, Side
from World import World

PR2_SERVICE_PREFIX = 'pr2_'
SERVICE_FK_INFO_POSTFIX = '_arm_kinematics_simple/get_fk_solver_info'
SERVICE_FK_POSTFIX = '_arm_kinematics_simple/get_fk'
BASE_LINK = 'base_link'

class Arm:
    ''' Interfacing with one arm for controlling mode and action execution'''

    _is_autorelease_on = False

    def __init__(self, arm_index):
        self.arm_index = arm_index

        self.arm_mode = ArmMode.HOLD
        self.gripper_state = None

        self.gripper_joint_name = self._side_prefix() + '_gripper_joint'
        self.ee_name = self._side_prefix() + '_wrist_roll_link'
        self.joint_names = [self._side_prefix() + '_shoulder_pan_joint',
                       self._side_prefix() + '_shoulder_lift_joint',
                       self._side_prefix() + '_upper_arm_roll_joint',
                       self._side_prefix() + '_elbow_flex_joint',
                       self._side_prefix() + '_forearm_roll_joint',
                       self._side_prefix() + '_wrist_flex_joint',
                       self._side_prefix() + '_wrist_roll_joint']

        self.all_joint_names = []
        self.all_joint_poses = []

        self.last_ee_pose = None
        self.movement_buffer_size = 40
        self.last_unstable_time = rospy.Time.now()
        self.arm_movement = []

        rospy.loginfo('Initializing ' + self._side() + ' arm.')

        switch_controller = 'pr2_controller_manager/switch_controller'
        rospy.wait_for_service(switch_controller)
        self.switch_service = rospy.ServiceProxy(switch_controller,
                                                 SwitchController)
        rospy.loginfo('Got response form the switch controller for '
                      + self._side() + ' arm.')

        # Create a trajectory action client
        traj_controller_name = (self._side_prefix()
                    + '_arm_controller/joint_trajectory_action')
        self.traj_action_client = SimpleActionClient(
                        traj_controller_name, JointTrajectoryAction)
        self.traj_action_client.wait_for_server()
        rospy.loginfo('Got response form trajectory action server for '
                      + self._side() + ' arm.')

        # Set up Inversse Kinematics
        self.ik_srv = None
        self.ik_request = None
        self.ik_joints = None
        self.ik_limits = None
        self._setup_ik()

        # Set up Forward Kinematics
        self.fk_srv = None
        self.fk_request = None
        self._setup_fk()        

        gripper_name = (self._side_prefix() +
                        '_gripper_controller/gripper_action')
        self.gripper_client = SimpleActionClient(gripper_name,
                                                    Pr2GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo('Got response form gripper server for '
                      + self._side() + ' arm.')

        filter_srv_name = '/trajectory_filter/filter_trajectory'
        rospy.wait_for_service(filter_srv_name)
        self.filter_service = rospy.ServiceProxy(filter_srv_name,
                                                 FilterJointTrajectory)
        rospy.loginfo('Filtering service has responded for ' +
                      self._side() + ' arm.')

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)

    def _setup_ik(self):
        '''Sets up services for inverse kinematics'''
        ik_info_srv_name = ('pr2_' + self._side() +
                            '_arm_kinematics_simple/get_ik_solver_info')
        ik_srv_name = 'pr2_' + self._side() + '_arm_kinematics_simple/get_ik'
        rospy.wait_for_service(ik_info_srv_name)
        ik_info_srv = rospy.ServiceProxy(ik_info_srv_name,
                                         GetKinematicSolverInfo)
        solver_info = ik_info_srv()
        rospy.loginfo('IK info service has responded for '
                      + self._side() + ' arm.')
        rospy.wait_for_service(ik_srv_name)
        self.ik_srv = rospy.ServiceProxy(ik_srv_name,
                                         GetPositionIK, persistent=True)
        rospy.loginfo('IK service has responded for ' + self._side() + ' arm.')

        # Set up common parts of an IK request
        self.ik_request = GetPositionIKRequest()
        self.ik_request.timeout = rospy.Duration(4.0)
        self.ik_joints = solver_info.kinematic_solver_info.joint_names
        self.ik_limits = solver_info.kinematic_solver_info.limits
        ik_links = solver_info.kinematic_solver_info.link_names

        request = self.ik_request.ik_request
        request.ik_link_name = ik_links[0]
        request.pose_stamped.header.frame_id = 'base_link'
        request.ik_seed_state.joint_state.name = self.ik_joints
        request.ik_seed_state.joint_state.position = [0] * len(self.ik_joints)


    def _setup_fk(self):
        '''Sets up services for forward kinematics.'''
        side = self._side()
        # Get FK info service.
        fk_info_srv_name = PR2_SERVICE_PREFIX + side + SERVICE_FK_INFO_POSTFIX
        rospy.wait_for_service(fk_info_srv_name)
        fk_info_srv = rospy.ServiceProxy(
        fk_info_srv_name, GetKinematicSolverInfo)
        ks_info = fk_info_srv().kinematic_solver_info
        rospy.loginfo('FK info service has responded for ' + side + ' arm.')
        # Get FK service.
        fk_srv_name = PR2_SERVICE_PREFIX + side + SERVICE_FK_POSTFIX
        rospy.wait_for_service(fk_srv_name)
        self.fk_srv = rospy.ServiceProxy(
        fk_srv_name, GetPositionFK, persistent=True)
        rospy.loginfo('FK service has responded for ' + side + ' arm.')
        # Set up common parts of an FK request.
        self.fk_request = GetPositionFKRequest()
        self.fk_request.header.frame_id = BASE_LINK
        self.fk_request.fk_link_names = ks_info.link_names
        self.fk_request.robot_state.joint_state.name = ks_info.joint_names

    def _side(self):
        '''Returns the word right or left depending on arm side'''
        if (self.arm_index == Side.RIGHT):
            return 'right'
        elif (self.arm_index == Side.LEFT):
            return 'left'

    def _side_prefix(self):
        ''' Returns the letter r or l depending on arm side'''
        side = self._side()
        return side[0]

    def get_ee_state(self, ref_frame='base_link'):
        ''' Returns end effector pose for the arm'''
        try:
            time = World.tf_listener.getLatestCommonTime(ref_frame,
                                                         self.ee_name)
            (position, orientation) = World.tf_listener.lookupTransform(
                                                ref_frame, self.ee_name, time)
            ee_pose = Pose()
            ee_pose.position = Point(position[0], position[1], position[2])
            ee_pose.orientation = Quaternion(orientation[0], orientation[1],
                                             orientation[2], orientation[3])
            return ee_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn('Something wrong with transform request.')
            return None

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
        joint_states message is received'''
        self.lock.acquire()
        self.all_joint_names = msg.name
        self.all_joint_poses = msg.position
        self.lock.release()

    def get_joint_state(self, joint_names=None):
        '''Returns position for the requested or all arm joints'''
        if joint_names == None:
            joint_names = self.joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received!\n")
            return []

        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
        self.lock.release()
        return positions

    def _solve_ik(self, ee_pose, seed=None):
        '''Gets the IK solution for end effector pose'''

        self.ik_request.ik_request.pose_stamped.pose = ee_pose

        if seed == None:
            # If no see is specified for IK search, start search at midpoint
            seed = []
            for i in range(0, len(self.ik_joints)):
                seed.append((self.ik_limits[i].min_position +
                             self.ik_limits[i].max_position) / 2.0)
        self.ik_request.ik_request.ik_seed_state.joint_state.position = seed

        try:
            rospy.loginfo('Sending IK request.')
            response = self.ik_srv(self.ik_request)
            if(response.error_code.val == response.error_code.SUCCESS):
                return response.solution.joint_state.position
            else:
                return None
        except rospy.ServiceException:
            rospy.logerr('Exception while getting the IK solution.')
            return None

    def set_mode(self, mode):
        '''Releases or holds the arm by turning the controller on/off'''
        controller_name = self._side_prefix() + '_arm_controller'
        if mode == ArmMode.RELEASE:
            start_controllers = []
            stop_controllers = [controller_name]
            rospy.loginfo('Switching ' + str(self._side()) +
                          ' arm to the kinesthetic mode')
        elif mode == ArmMode.HOLD:
            start_controllers = [controller_name]
            stop_controllers = []
            rospy.loginfo('Switching ' + str(self._side()) +
                          ' to the Joint-control mode')
        else:
            rospy.logwarn('Unknown mode ' + str(mode) +
                          '. Keeping the current mode.')
            return

        try:
            self.switch_service(start_controllers, stop_controllers, 1)
            self.arm_mode = mode
        except rospy.ServiceException:
            rospy.logerr("Service did not process request")

    def get_mode(self):
        '''Returns the current arm mode (released/holding)'''
        return self.arm_mode

    def _send_gripper_command(self, pos=0.08, eff=30.0, wait=False):
        '''Sets the position of the gripper'''
        command = Pr2GripperCommandGoal()
        command.command.position = pos
        command.command.max_effort = eff
        self.gripper_client.send_goal(command)
        if wait:
            self.gripper_client.wait_for_result(rospy.Duration(10.0))

    def is_gripper_moving(self):
        ''' Whether or not the gripper is in the process of opening/closing'''
        return (self.gripper_client.get_state() == GoalStatus.ACTIVE or
                self.gripper_client.get_state() == GoalStatus.PENDING)

    def is_gripper_at_goal(self):
        ''' Whether or not the gripper has reached its goal'''
        return (self.gripper_client.get_state() == GoalStatus.SUCCEEDED)

    def get_gripper_state(self):
        '''Returns current gripper state'''
        return self.gripper_state

    def check_gripper_state(self, joint_name=None):
        '''Checks gripper state at the hardware level'''
        if (joint_name == None):
            joint_name = self.gripper_joint_name
        gripper_pos = self.get_joint_state([joint_name])
        if gripper_pos != []:
            if gripper_pos[0] > 0.078:
                self.gripper_state = GripperState.OPEN
            else:
                self.gripper_state = GripperState.CLOSED
        else:
            rospy.logwarn('Could not update the gripper state.')

    def open_gripper(self, pos=0.08, eff=30.0, wait=False):
        '''Opens gripper'''
        self._send_gripper_command(pos, eff, wait)
        self.gripper_state = GripperState.OPEN

    def close_gripper(self, pos=0.0, eff=30.0, wait=False):
        '''Closes gripper'''
        self._send_gripper_command(pos, eff, wait)
        self.gripper_state = GripperState.CLOSED

    def set_gripper(self, gripper_state):
        '''Sets gripper to the desired state'''
        if (gripper_state == GripperState.CLOSED):
            self.close_gripper()
        elif (gripper_state == GripperState.OPEN):
            self.open_gripper()

    # *******

    def execute_joint_traj(self, joint_trajectory, timing):
        '''Moves the arm through the joint sequence'''

        # First, do filtering on the trajectory to fix the velocities
        trajectory = JointTrajectory()

        # Initialize the server
        # When to start the trajectory: 0.1 seconds from now
        trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trajectory.joint_names = self.joint_names

         ## Add all frames of the trajectory as way points
        for i in range(len(timing)):
            positions = joint_trajectory[i].joint_pose
            #experimenting with the velocity settings.
            velocities = [0] * len(positions) 

            # for smoothing the trajectory:
            # print timing[len(timing)-1].to_sec()
            # print timing[0].to_sec()
            # print len(timing)

            # print timing[0]
            # print timing[len(timing)-1]

            new_timing = timing[0] + rospy.Duration(i*(timing[len(timing)-1].to_sec() - timing[0].to_sec())/len(timing))


              # Add frames to the trajectory
            trajectory.points.append(JointTrajectoryPoint(positions=positions,
                                     velocities=velocities,
                                     time_from_start=timing[i]))
            # use the following trajectory if uniformal time slot distribution is prefered:
            # trajectory.points.append(JointTrajectoryPoint(positions=positions,
            #                          velocities=velocities,
            #                          time_from_start=new_timing))

        # added by Joseph for smoothing the replay trajectory:
        # added_velocities = [[0]*7]*(len(timing) - 1)

        # for i in range(len(timing) - 1): 

        #     time_diff= subtract(timing[i+1].to_sec(), timing[i].to_sec())
        #     joint_diff = subtract(trajectory.points[i+1].positions, trajectory.points[i].positions)

        #     added_velocities[i] = [x/time_diff for x in joint_diff]

        #     trajectory.points[i].velocities = added_velocities[i]


        # send the recorded trajectory to the filter
        # output = self.filter_service(trajectory=trajectory,
        #                              allowed_time=rospy.Duration.from_sec(20))

        # if(output.error_code.val == output.error_code.SUCCESS):

        #     rospy.loginfo('Trajectory for arm ' + str(self.arm_index) + 
        #                   ' has been filtered.')
        traj_goal = JointTrajectoryGoal()   

        #traj_goal.trajectory = output.trajectory
        
        traj_goal.trajectory = trajectory
        traj_goal.trajectory.header.stamp = (rospy.Time.now() +
                                            rospy.Duration(0.1))
        traj_goal.trajectory.joint_names = self.joint_names

        n_points = len(traj_goal.trajectory.points)
                        
            

        # added for reducing the jerky motion.
        # added_velocities = [[0]*7]*(n_points - 1)

        for i in range(n_points - 1): 

            time_diff = subtract(traj_goal.trajectory.points[i+1].time_from_start.to_sec(), traj_goal.trajectory.points[i].time_from_start.to_sec())
        #     joint_diff = subtract(traj_goal.trajectory.points[i+1].positions, traj_goal.trajectory.points[i].positions)
            
        #     added_velocities[i] = [x/time_diff for x in joint_diff]

        #     traj_goal.trajectory.points[i].velocities = added_velocities[i]
        #    print time_diff

        # Sends the goal to the trajectory server
        # DISABLING FOR DEBUGGING

        self.traj_action_client.send_goal(traj_goal)
        return True

        # else:
        #     rospy.logwarn('Trajectory filtering failed.')
        #     print output.error_code.val
        #     return False

    def move_to_joints(self, joints, time_to_joint):
        '''Moves the arm to the desired joints'''
        # Setup the goal
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() +
                                             rospy.Duration(0.1))
        traj_goal.trajectory.joint_names = self.joint_names
        velocities = [0] * len(joints)
        traj_goal.trajectory.points.append(JointTrajectoryPoint(    
                        positions=joints,
                        velocities=velocities,
                        time_from_start=rospy.Duration(time_to_joint)))

        # Client sends the goal to the Server
        self.traj_action_client.send_goal(traj_goal)

    def is_executing(self):
        '''Whether or not there is an ongoing action execution on the arm'''
        return (self.traj_action_client.get_state() == GoalStatus.ACTIVE
                or self.traj_action_client.get_state() == GoalStatus.PENDING)

    def is_successful(self):
        '''Whetehr the execution succeeded'''
        return (self.traj_action_client.get_state() == GoalStatus.SUCCEEDED)

    def get_ik_for_ee(self, ee_pose, seed):
        ''' Finds the IK solution for given end effector pose.

        Note that though seed is not explicitly allowed to be None in
        this method, it often is in code that this calls. If None is
        passed in for the seed, None may be returned.

        Args:
        ee_pose (Pose): Pose to solve IK for.
        seed ([float]): Seven-element list of arm joint positions.

        Returns:
        [float]: Seven-element list of arm joint positions. If seed
        passed in is None, and no IK solution is found, None
        could be returned.
        '''
        
        joints = self._solve_ik(ee_pose, seed)
        # If our seed did not work, try once again with the default
        # seed.
        if joints is None:
            rospy.logdebug(
                'Could not find IK solution with preferred seed, will try ' +
                'default seed.')
            joints = self._solve_ik(ee_pose)

            if joints is None:
                rospy.logdebug('IK out of bounds, considering the seed directly.')
                # IK failed, but let's see if FK with the passed seed will
                # give us a pose close enough to the ee_pose that it's
                # usable.
                fk_pose = self.get_fk_for_joints(seed)
                if Arm.get_distance_bw_poses(ee_pose, fk_pose) <  0.02:
                    joints = seed
                    rospy.logdebug('IK out of bounds, but FK close; using seed.')

        if joints is not None:
            rollover = array((array(joints) - array(seed)) / pi, int)
            joints -= ((rollover + (sign(rollover) + 1) / 2) / 2) * 2 * pi

        return joints

    def get_fk_for_joints(self, joints):
        '''Finds the FK solution (EE pose) for given joint positions.
        Args:
            joints ([float]) Seven-element list of arm joint positions.
        Returns:
            Pose|None: The arm's pose when its joints are at the
            specified values, or None if there was a problem finding
        an FK solution
        '''
        self.fk_request.robot_state.joint_state.position = joints
        pose = None
        try:
            resp = self.fk_srv(self.fk_request)
            pose_idx = resp.fk_link_names.index(self.ee_name)
            pose = resp.pose_stamped[pose_idx].pose
        except rospy.ServiceException:
            rospy.logwarn(
                'There was an error with the FK request for joints: ' +
                    str(joints))
        finally:
            return pose

    @staticmethod
    def get_distance_bw_poses(pose0, pose1):
        '''Returns the dissimilarity between two end-effector poses'''
        w_pos = 1.0
        w_rot = 0.2
        pos0 = array((pose0.position.x, pose0.position.y, pose0.position.z))
        pos1 = array((pose1.position.x, pose1.position.y, pose1.position.z))
        rot0 = array((pose0.orientation.x, pose0.orientation.y,
                      pose0.orientation.z, pose0.orientation.w))
        rot1 = array((pose1.orientation.x, pose1.orientation.y,
                      pose1.orientation.z, pose1.orientation.w))
        pos_dist = w_pos * norm(pos0 - pos1)
        rot_dist = w_rot * (1 - dot(rot0, rot1))

        if (pos_dist > rot_dist):
            dist = pos_dist
        else:
            dist = rot_dist
        return dist


    def reset_movement_history(self):
        ''' Clears the saved history of arm movements'''
        self.last_unstable_time = rospy.Time.now()
        self.arm_movement = []

    def get_movement(self):
        '''Returns cumulative movement in recent history'''
        return sum(self.arm_movement)

    def _record_arm_movement(self, reading):
        '''Records the sensed arm movement'''
        self.arm_movement = [reading] + self.arm_movement
        if (len(self.arm_movement) > self.movement_buffer_size):
            self.arm_movement = self.arm_movement[0:self.movement_buffer_size]

    def _is_arm_moved_while_holding(self):
        '''Checks if user is trying to move the arm while it is stiff'''
        threshold = 0.02
        if (self.get_mode() == ArmMode.HOLD
                and (len(self.arm_movement) == self.movement_buffer_size)
                and (self.get_movement() > threshold)):
            return True
        return False

    def _is_arm_stable_while_released(self):
        '''Checks if the arm has been stable while being released'''
        movement_threshold = 0.02
        time_threshold = rospy.Duration(5.0)
        is_arm_stable = (self.get_movement() < movement_threshold)
        if (not is_arm_stable or self.get_mode() == ArmMode.HOLD):
            self.last_unstable_time = rospy.Time.now()
            return False
        else:
            if (rospy.Time.now() - self.last_unstable_time) > time_threshold:
                return True
            else:
                return False

    def update(self, is_executing):
        ''' Periodical update for one arm'''
        ee_pose = self.get_ee_state()
        if (ee_pose != None and self.last_ee_pose != None):
            self._record_arm_movement(Arm.get_distance_bw_poses(ee_pose,
                                                        self.last_ee_pose))
        self.last_ee_pose = ee_pose

        if (not is_executing and Arm._is_autorelease_on):
            if (self._is_arm_moved_while_holding()):
                rospy.loginfo('Automatically releasing arm.')
                self.set_mode(ArmMode.RELEASE)

            if (self._is_arm_stable_while_released()):
                rospy.loginfo('Automatically holding arm.')
                self.set_mode(ArmMode.HOLD)
