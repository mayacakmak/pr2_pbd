'''Control of the two arms for action execution'''
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

import time
import threading
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import ActionStep, Side
from pr2_pbd_interaction.msg import ExecutionStatus
from pr2_social_gaze.msg import GazeGoal
from geometry_msgs.msg import Pose, Point
from Response import Response
from World import World
from Arm import Arm, ArmMode


class Arms:
    '''Class for things related to moving arms'''
    arms = []

    def __init__(self):
        r_arm = Arm(Side.RIGHT)
        l_arm = Arm(Side.LEFT)
        Arms.arms = [r_arm, l_arm]
        self.attended_arm = -1
        self.action = None
        self.preempt = False
	self.z_offset = 0

        rospy.loginfo('Arms have been initialized.')

        Arms.arms[0].set_mode(ArmMode.HOLD)
        Arms.arms[1].set_mode(ArmMode.HOLD)
        Arms.arms[0].check_gripper_state()
        Arms.arms[1].check_gripper_state()
        Arms.arms[Side.RIGHT].close_gripper()
        Arms.arms[Side.LEFT].close_gripper()
        self.status = ExecutionStatus.NOT_EXECUTING

    @staticmethod
    def set_arm_mode(arm_index, mode):
        '''Set arm to stiff or relaxed'''
        if (mode == Arms.arms[arm_index].arm_mode):
            # Already in that mode
            return False
        else:
            Arms.arms[arm_index].set_mode(mode)
            return True

    @staticmethod
    def set_gripper_state(arm_index, gripper_state):
        '''Set gripper to open or closed'''
        if (gripper_state == Arms.get_gripper_state(arm_index)):
            # Already in that mode
            return False
        else:
            if (gripper_state == GripperState.OPEN):
                Arms.arms[arm_index].open_gripper()
            else:
                Arms.arms[arm_index].close_gripper()
        return True

    def is_executing(self):
        '''Whether or not there is an ongoing action execution'''
        return (self.status == ExecutionStatus.EXECUTING)

    def start_execution(self, action, z_offset=0):
        ''' Starts execution of an action'''
        # This will take long, create a thread
        self.action = action.copy()
        self.preempt = False
	self.z_offset = z_offset
        thread = threading.Thread(group=None, target=self.execute_action,
                                  name='skill_execution_thread')
        thread.start()

    def stop_execution(self):
        '''Preempts an ongoing execution'''
        self.preempt = True

    def solve_ik_for_action(self):
        '''Computes joint positions for all end-effector poses
        in an action'''

        # Go over steps of the action
        for i in range(self.action.n_frames()):
            # For each step check step type
            # If arm target action
            if (self.action.seq.seq[i].type == ActionStep.ARM_TARGET):
                # Find frames that are relative and convert to absolute

                r_arm, has_solution_r = Arms.solve_ik_for_arm(0,
                                        self.action.seq.seq[i].armTarget.rArm,
					self.z_offset)
                l_arm, has_solution_l = Arms.solve_ik_for_arm(1,
                                        self.action.seq.seq[i].armTarget.lArm,
					self.z_offset)

                self.action.seq.seq[i].armTarget.rArm = r_arm
                self.action.seq.seq[i].armTarget.lArm = l_arm
                if (not has_solution_r) or (not has_solution_l):
                    return False

            if (self.action.seq.seq[i].type == ActionStep.ARM_TRAJECTORY):
                n_frames = len(self.action.seq.seq[i].armTrajectory.timing)
                for j in range(n_frames):
                    r_arm, has_solution_r = Arms.solve_ik_for_arm(0,
                            self.action.seq.seq[i].armTrajectory.r_arm[j],
			    self.z_offset)
                    l_arm, has_solution_l = Arms.solve_ik_for_arm(1,
                            self.action.seq.seq[i].armTrajectory.l_arm[j],
			    self.z_offset)
                    self.action.seq.seq[i].armTrajectory.r_arm[j] = r_arm
                    self.action.seq.seq[i].armTrajectory.l_arm[j] = l_arm
                    if (not has_solution_r) or (not has_solution_l):
                        return False
        return True

    @staticmethod
    def solve_ik_for_arm(arm_index, arm_state, z_offset=0):
        '''Finds an  IK solution for a particular arm pose'''
        # We need to find IK only if the frame is relative to an object
        if (arm_state.refFrame == ArmState.OBJECT):
	    #rospy.loginfo('solve_ik_for_arm: Arm ' + str(arm_index) + ' is relative')
            solution = ArmState()
            target_pose = World.transform(arm_state.ee_pose,
                            arm_state.refFrameObject.name, 'base_link')

	    target_pose.position.z = target_pose.position.z + z_offset

            target_joints = Arms.arms[arm_index].get_ik_for_ee(target_pose,
                                            arm_state.joint_pose)
            if (target_joints == None):
                rospy.logerr('No IK for relative end-effector pose.')
                return solution, False
            else:
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(target_pose.position,
                                        target_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        elif (arm_state.refFrame == ArmState.ROBOT_BASE):
	    #rospy.loginfo('solve_ik_for_arm: Arm ' + str(arm_index) + ' is absolute')
	    pos = arm_state.ee_pose.position
	    target_position = Point(pos.x, pos.y, pos.z + z_offset)
	    target_pose = Pose(target_position, arm_state.ee_pose.orientation)
            target_joints = Arms.arms[arm_index].get_ik_for_ee(target_pose,
                                                    arm_state.joint_pose)
            if (target_joints == None):
                rospy.logerr('No IK for absolute end-effector pose.')
                return arm_state, False
            else:
                solution = ArmState()
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(arm_state.ee_pose.position,
                                        arm_state.ee_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        else:
            return arm_state, True

    @staticmethod
    def is_condition_met(dummy):
        ''' Checks if the given precondition is currently met'''
        # TO-DO
        return True

    def start_move_to_pose(self, arm_state, arm_index):
        '''Creates a thread for moving to a target pose'''
        self.preempt = False
        thread = threading.Thread(group=None, target=self.move_to_pose,
                    args=(arm_state, arm_index,),
                    name='move_to_arm_state_thread')
        thread.start()

    def move_to_pose(self, arm_state, arm_index):
        '''The thread function that makes the arm move to
        a target end-effector pose'''
        rospy.loginfo('Started thread to move arm ' + str(arm_index))
        self.status = ExecutionStatus.EXECUTING
        solution, has_solution = Arms.solve_ik_for_arm(arm_index, arm_state)

        if (has_solution):
            if (arm_index == 0):
                is_successful = self.move_to_joints(solution, None)
            else:
                is_successful = self.move_to_joints(None, solution)

            if (is_successful):
                self.status = ExecutionStatus.SUCCEEDED
            else:
                self.status = ExecutionStatus.OBSTRUCTED
        else:
            self.status = ExecutionStatus.NO_IK

    def execute_action(self):
        ''' Function to replay the demonstrated two-arm action
        of type ProgrammedAction'''
        self.status = ExecutionStatus.EXECUTING
        # Check if the very first precondition is met
        action_step = self.action.get_step(0)
        if (not Arms.is_condition_met(action_step.preCond)):
            rospy.logwarn('First precond is not met, first make sure ' +
                          'the robot is ready to execute action ' +
                          '(hand object or free hands).')
            self.status = ExecutionStatus.CONDITION_ERROR
        else:
            # Check that all parts of the action are reachable
            if (not self.solve_ik_for_action()):
                rospy.logwarn('Problems in finding IK solutions...')
                self.status = ExecutionStatus.NO_IK
            else:
                Arms.set_arm_mode(0, ArmMode.HOLD)
                Arms.set_arm_mode(1, ArmMode.HOLD)
                self._loop_through_action_steps()

            Arms.arms[0].reset_movement_history()
            Arms.arms[1].reset_movement_history()

            if self.status == ExecutionStatus.EXECUTING:
                self.status = ExecutionStatus.SUCCEEDED
                rospy.loginfo('Skill execution has succeeded.')

    def _loop_through_action_steps(self):
        ''' Goes through the steps of the current action'''

        # Go over steps of the action
        for i in range(self.action.n_frames()):
            rospy.loginfo('Executing step ' + str(i))
            action_step = self.action.get_step(i)

            # Check that preconditions are met
            if (not Arms.is_condition_met(action_step.preCond)):
                rospy.logwarn('Preconditions of action step ' + str(i) +
                              ' are not satisfied. Aborting.')
                self.status = ExecutionStatus.PREEMPTED
                break
            else:
                if (not self._execute_action_step(action_step)):
                    break

                # Check that postconditions are met
                if (Arms.is_condition_met(action_step.postCond)):
                    rospy.loginfo('Post-conditions of the action are met.')
                else:
                    rospy.logwarn('Post-conditions of action step ' +
                                  str(i) + ' are not satisfied. Aborting.')
                    self.status = ExecutionStatus.PREEMPTED
                    break

            if (self.preempt):
                rospy.logwarn('Execution preempted by user.')
                self.status = ExecutionStatus.PREEMPTED
                break

            rospy.loginfo('Step ' + str(i) + ' of action is complete.')

    def _execute_action_step(self, action_step):
        '''Executes the motion part of an action step'''
        # For each step check step type
        # If arm target action
        if (action_step.type == ActionStep.ARM_TARGET):
            rospy.loginfo('Will perform arm target action step.')

            if (not self.move_to_joints(action_step.armTarget.rArm,
                                        action_step.armTarget.lArm)):
                self.status = ExecutionStatus.OBSTRUCTED
                return False

        # If arm trajectory action
        elif (action_step.type == ActionStep.ARM_TRAJECTORY):

            rospy.loginfo('Will perform arm trajectory action step.')

            # First move to the start frame
            if (not self.move_to_joints(action_step.armTrajectory.r_arm[0],
                                        action_step.armTrajectory.l_arm[0])):
                self.status = ExecutionStatus.OBSTRUCTED
                return False

            #  Then execute the trajectory
            Arms.arms[0].exectute_joint_traj(action_step.armTrajectory.r_arm,
                                             action_step.armTrajectory.timing)
            Arms.arms[1].exectute_joint_traj(action_step.armTrajectory.l_arm,
                                             action_step.armTrajectory.timing)

            # Wait until both arms complete the trajectory
            while((Arms.arms[0].is_executing() or Arms.arms[1].is_executing())
                  and not self.preempt):
                time.sleep(0.01)
            rospy.loginfo('Trajectory complete.')

            # Verify that both arms succeeded
            if ((not Arms.arms[0].is_successful()) or
                (not Arms.arms[1].is_successful())):
                rospy.logwarn('Aborting execution; ' +
                              'arms failed to follow trajectory.')
                self.status = ExecutionStatus.OBSTRUCTED
                return False

        # If hand action do it for both sides
        if (action_step.gripperAction.rGripper !=
                            Arms.arms[0].get_gripper_state()):
            rospy.loginfo('Will perform right gripper action ' +
                          str(action_step.gripperAction.rGripper))
            Arms.arms[0].set_gripper(action_step.gripperAction.rGripper)
            Response.perform_gaze_action(GazeGoal.FOLLOW_RIGHT_EE)

        if (action_step.gripperAction.lGripper !=
                            Arms.arms[1].get_gripper_state()):
            rospy.loginfo('Will perform LEFT gripper action ' +
                          str(action_step.gripperAction.lGripper))
            Arms.arms[1].set_gripper(action_step.gripperAction.lGripper)
            Response.perform_gaze_action(GazeGoal.FOLLOW_LEFT_EE)

        # Wait for grippers to be done
        while(Arms.arms[0].is_gripper_moving() or
              Arms.arms[1].is_gripper_moving()):
            time.sleep(0.01)
        rospy.loginfo('Hands done moving.')

        # Verify that both grippers succeeded
        if ((not Arms.arms[0].is_gripper_at_goal()) or
            (not Arms.arms[1].is_gripper_at_goal())):
            rospy.logwarn('Hand(s) did not fully close or open!')

        return True

    @staticmethod
    def _get_time_to_pose(pose, arm_index):
        ''' Returns the time to get to an arm pose'''
        if (pose == None):
            rospy.logwarn('Arm ' + str(arm_index) + 'will not move.')
            return None
        else:
            time_to_pose = Arms._get_time_bw_poses(
                            Arms.arms[arm_index].get_ee_state(),
                            pose.ee_pose)
            rospy.loginfo('Duration until next frame for arm ' +
                          str(arm_index) + ': ' + str(time_to_pose))
            return time_to_pose

    def move_to_joints(self, r_arm, l_arm):
        '''Makes the arms move to indicated joint poses'''
        time_to_r_pose = Arms._get_time_to_pose(r_arm, 0)
        time_to_l_pose = Arms._get_time_to_pose(l_arm, 1)

        #  If both arms are moving adjust velocities and find most moving arm
        is_r_moving = (time_to_r_pose != None)
        is_l_moving = (time_to_l_pose != None)
        if (not is_r_moving):
            Response.look_at_point(l_arm.ee_pose.position)
        elif (not is_l_moving):
            Response.look_at_point(r_arm.ee_pose.position)
        else:
            if (time_to_r_pose > time_to_l_pose):
                time_to_l_pose = time_to_r_pose
                Response.look_at_point(r_arm.ee_pose.position)
            else:
                time_to_r_pose = time_to_l_pose
                Response.look_at_point(l_arm.ee_pose.position)

        #  Move arms to target
        if (is_r_moving):
            Arms.arms[0].move_to_joints(r_arm.joint_pose, time_to_r_pose)
        if (is_l_moving):
            Arms.arms[1].move_to_joints(l_arm.joint_pose, time_to_l_pose)

        # Wait until both arms complete the trajectory
        while((Arms.arms[0].is_executing() or
               Arms.arms[1].is_executing()) and not self.preempt):
            time.sleep(0.01)
        rospy.loginfo('Arms reached target.')

        # Verify that both arms succeeded
        if ((not Arms.arms[0].is_successful() and is_r_moving) or
            (not Arms.arms[1].is_successful() and is_l_moving)):
            rospy.logwarn('Aborting because arms failed to move to pose.')
            return False
        else:
            return True

    @staticmethod
    def _get_most_moving_arm():
        '''Determines which of the two arms has moved more
        in the recent past'''
        threshold = 0.02
        if (Arms.arms[0].get_movement() < threshold and
            Arms.arms[1].get_movement() < threshold):
            return -1
        elif (Arms.arms[0].get_movement() < threshold):
            return 1
        else:
            return 0

    @staticmethod
    def get_joint_state(arm_index):
        '''Get joint poritions'''
        return Arms.arms[arm_index].get_joint_state()

    @staticmethod
    def get_gripper_state(arm_index):
        ''' Get gripper status on the indicated side'''
        return Arms.arms[arm_index].get_gripper_state()

    @staticmethod
    def get_ee_state(arm_index):
        ''' Get pose of the end-effector on the indicated side'''
        return Arms.arms[arm_index].get_ee_state()

    @staticmethod
    def _get_time_bw_poses(pose0, pose1, velocity=0.2):
        '''Determines how much time should be allowed for
        moving between two poses'''
        dist = Arm.get_distance_bw_poses(pose0, pose1)
        duration = dist / velocity
        if duration < 0.5:
            duration = 0.5
        return duration

    def update(self):
        '''Periodic update for the two arms'''
        Arms.arms[0].update(self.is_executing())
        Arms.arms[1].update(self.is_executing())

        moving_arm = Arms._get_most_moving_arm()
        if (moving_arm != self.attended_arm and not self.is_executing()):
            if (moving_arm == -1):
                Response.perform_gaze_action(GazeGoal.LOOK_FORWARD)
            elif (moving_arm == 0):
                Response.perform_gaze_action(GazeGoal.FOLLOW_RIGHT_EE)
            else:
                Response.perform_gaze_action(GazeGoal.FOLLOW_LEFT_EE)
            self.attended_arm = moving_arm
