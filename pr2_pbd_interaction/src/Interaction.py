'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import rospy
import time
from visualization_msgs.msg import MarkerArray

# Local stuff
from World import World
from RobotSpeech import RobotSpeech
from Session import Session
from Response import Response
from Arms import Arms
from Arm import ArmMode
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import ActionStep, ArmTarget, Object
from pr2_pbd_interaction.msg import GripperAction, ArmTrajectory
from pr2_pbd_interaction.msg import ExecutionStatus, GuiCommand
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal


class DemoState:
    READY_TO_TAKE="READY_TO_TAKE"
    READY_FOR_DEMO="READY_FOR_DEMO"
    TOOL_NOT_RECOGNIZED="TOOL_NOT_RECOGNIZED"

class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        self.arms = Arms()
        self.world = World()
        self.session = Session(object_list=self.world.get_frame_list(),
                               is_debug=True)
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
                                              MarkerArray)

	self._demo_state = None

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self.responses = {
            Command.TEST_MICROPHONE: Response(Interaction.empty_response,
                                [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD]),
            Command.TAKE_TOOL: Response(self.take_tool, 0),
            Command.RELEASE_TOOL: Response(self.open_hand, 0),
            Command.DETECT_SURFACE: Response(self.record_object_pose, None),
            Command.START_RECORDING: Response(self.start_recording, None),
            Command.STOP_RECORDING: Response(self.stop_recording, None),
            Command.REPLAY_DEMONSTRATION: Response(self.execute_action, None),
            Command.SAVE_ARM_POSE: Response(self.save_arm_pose, None)
        }

        rospy.loginfo('Will wait until arms ready to respond.')
        while ((self.arms.get_ee_state(0) is None) or 
        		(self.arms.get_ee_state(1) is None)):
        	time.sleep(0.1)

        rospy.loginfo('Starting to move to the initial pose.')

        self._move_to_arm_pose('take', 0)
        self._move_to_arm_pose('take', 1)

	self._demo_state = DemoState.READY_TO_TAKE
        rospy.loginfo('Interaction initialized.')

    def load_known_arm_poses(self):
	'''This loads important poses from the hard drive'''
	# TODO
	pass

    def open_hand(self, arm_index):
        '''Opens gripper on the indicated side'''
        if self.arms.set_gripper_state(arm_index, GripperState.OPEN):
            speech_response = Response.open_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.OPEN)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_open_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def take_tool(self, arm_index):
    	self.close_hand(arm_index)
    	self._move_to_arm_pose('initial', arm_index)

    def close_hand(self, arm_index):
        '''Closes gripper on the indicated side'''
        if Arms.set_gripper_state(arm_index, GripperState.CLOSED):
            speech_response = Response.close_responses[arm_index]
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_closed_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def relax_arm(self, arm_index):
        '''Relaxes arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.RELEASE):
            return [Response.release_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_released_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def freeze_arm(self, arm_index):
        '''Stiffens arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.HOLD):
            return [Response.hold_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_holding_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def edit_action(self, dummy=None):
        '''Goes back to edit mode'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                return [RobotSpeech.ALREADY_EDITING, GazeGoal.SHAKE]
            else:
                Interaction._is_programming = True
                return [RobotSpeech.SWITCH_TO_EDIT_MODE, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def save_action(self, dummy=None):
        '''Goes out of edit mode'''
        self.session.save_current_action()
        Interaction._is_programming = False
        return [RobotSpeech.ACTION_SAVED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def create_action(self, dummy=None):
        '''Creates a new empty action'''
        self._move_to_arm_pose('take', 0)
        self.world.clear_all_objects()
        self.session.new_action()
        Interaction._is_programming = True
        return [RobotSpeech.SKILL_CREATED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def next_action(self, dummy=None):
        '''Switches to next action'''
        if (self.session.n_actions() > 0):
            if self.session.next_action(self.world.get_frame_list()):
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_NEXT_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def previous_action(self, dummy=None):
        '''Switches to previous action'''
        if (self.session.n_actions() > 0):
            if self.session.previous_action(self.world.get_frame_list()):
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_PREV_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_all_steps(self, dummy=None):
        '''Deletes all steps in the current action'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_frames() > 0:
                    self.session.clear_current_action()
                    return [RobotSpeech.SKILL_CLEARED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, None]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]
    
    def _resume_all_steps(self):
        '''Resumes all steps after clearing'''
        self.session.undo_clear()
        return [RobotSpeech.ALL_POSES_RESUMED, GazeGoal.NOD]

    def _resume_last_step(self):
        '''Resumes last step after deleting'''
        self.session.resume_deleted_step()
        return [RobotSpeech.POSE_RESUMED, GazeGoal.NOD]

    def stop_execution(self, dummy=None):
        '''Stops ongoing execution'''
        if (self.arms.is_executing()):
            self.arms.stop_execution()
            return [RobotSpeech.STOPPING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]

    def save_gripper_step(self, arm_index, gripper_state):
        '''Saves an action step that involves a gripper state change'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ActionStep()
                step.type = ActionStep.ARM_TARGET
                step.armTarget = ArmTarget(states[0], states[1], 0.2, 0.2)
                actions = [self.arms.get_gripper_state(0),
                           self.arms.get_gripper_state(1)]
                actions[arm_index] = gripper_state
                step.gripperAction = GripperAction(actions[0], actions[1])
                self.session.add_step_to_action(step,
                                                self.world.get_frame_list())

    def start_recording(self, dummy=None):
        '''Starts recording continuous motion'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if (not Interaction._is_recording_motion):
                    Interaction._is_recording_motion = True
                    Interaction._arm_trajectory = ArmTrajectory()
                    Interaction._trajectory_start_time = rospy.Time.now()

 		    if self.session.n_frames() > 0:
                        self.session.clear_current_action()

                    return [RobotSpeech.STARTED_RECORDING_MOTION,
                            GazeGoal.NOD]
                else:
                    return [RobotSpeech.ALREADY_RECORDING_MOTION,
                            GazeGoal.SHAKE]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def stop_recording(self, dummy=None):
        '''Stops recording continuous motion'''
        if (Interaction._is_recording_motion):
            Interaction._is_recording_motion = False
            traj_step = ActionStep()
            traj_step.type = ActionStep.ARM_TRAJECTORY

            waited_time = Interaction._arm_trajectory.timing[0]
            for i in range(len(Interaction._arm_trajectory.timing)):
                Interaction._arm_trajectory.timing[i] -= waited_time
                Interaction._arm_trajectory.timing[i] += rospy.Duration(0.1)
            '''If motion was relative, record transformed pose'''
            traj_step.armTrajectory = ArmTrajectory(
                Interaction._arm_trajectory.rArm[:],
                Interaction._arm_trajectory.lArm[:],
                Interaction._arm_trajectory.timing[:],
                Interaction._arm_trajectory.rRefFrame,
                Interaction._arm_trajectory.lRefFrame,
                Interaction._arm_trajectory.rRefFrameObject,
                Interaction._arm_trajectory.lRefFrameObject)
            traj_step.gripperAction = GripperAction(
                                        self.arms.get_gripper_state(0),
                                        self.arms.get_gripper_state(1))
                                        
            self.session.add_step_to_action(traj_step,
                                        self.world.get_frame_list())
            
            Interaction._arm_trajectory = None
            Interaction._trajectory_start_time = None
            
            return [RobotSpeech.STOPPED_RECORDING_MOTION + ' ' +
                    RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.MOTION_NOT_RECORDING, GazeGoal.SHAKE]

    def _fix_trajectory_ref(self):
        '''Makes the reference frame of continuous trajectories uniform'''
        (r_ref, _), (_, r_ref_obj) = self._find_dominant_ref(
                                                Interaction._arm_trajectory.rArm)
        (l_ref, _), (_, l_ref_obj) = self._find_dominant_ref(
                                                Interaction._arm_trajectory.lArm)
                                                
        for i in range(len(Interaction._arm_trajectory.timing)):
            Interaction._arm_trajectory.rArm[i] = World.convert_ref_frame(
                            Interaction._arm_trajectory.rArm[i],
                            r_ref, r_ref_obj)
            Interaction._arm_trajectory.lArm[i] = World.convert_ref_frame(
                            Interaction._arm_trajectory.lArm[i],
                            l_ref, l_ref_obj)
        
        Interaction._arm_trajectory.rRefFrame = r_ref
        Interaction._arm_trajectory.rRefFrameObject = r_ref_obj
        Interaction._arm_trajectory.lRefFrame = l_ref
        Interaction._arm_trajectory.lRefFrameObject = l_ref_obj

    def _find_dominant_ref(self, arm_traj):
        '''Finds the most dominant reference frame
        in a continuous trajectory'''
        def addEnt(dic, ent):
            key = (ent.refFrame, ent.refFrameObject.name)
            if (key in dic):
                dic[key] = (1 + dic[key][0], dic[key][1])
            else:
                dic[key] = (1, ent.refFrameObject)
            return dic

        cnt = reduce(addEnt, arm_traj, {})

        return reduce(lambda a, b: a if a[1][0] > b[1][0] else b, cnt.items())

    def _save_arm_to_trajectory(self):
        '''Saves current arm state into continuous trajectory'''
        if (Interaction._arm_trajectory != None):
            states =  self._get_arm_states() 	
            Interaction._arm_trajectory.rArm.append(states[0])
            Interaction._arm_trajectory.lArm.append(states[1])
            Interaction._arm_trajectory.timing.append(
                        rospy.Time.now() - Interaction._trajectory_start_time)

    def _move_to_arm_pose(self, pose_name, arm_index):
    	'''Moves the robot's arm to a pre-specified arm pose'''
    	action = self.session.pose_set[pose_name]
    	if action is None:
    		rospy.logwarn('Arm pose does not exist:' + pose_name)
    	else:
    		step = action.get_step(0)
    		if arm_index == 0:
    			self.arms.start_move_to_pose(step.armTarget.rArm, 0)
    			self.arms.set_gripper_state(0, step.gripperAction.rGripper)
    		else:
    			self.arms.start_move_to_pose(step.armTarget.lArm, 1)
    			self.arms.set_gripper_state(1, step.gripperAction.lGripper)

    		rospy.loginfo('Moved arm ' + str(arm_index) + ' to pose ' + pose_name)

    def save_arm_pose(self, dummy=None):
        '''Saves current arm state as an action step'''
        states = self._get_arm_states()
        step = ActionStep()
        step.type = ActionStep.ARM_TARGET
        step.armTarget = ArmTarget(states[0], states[1], 0.2, 0.2)
        step.gripperAction = GripperAction(
                                    self.arms.get_gripper_state(0),
                                    self.arms.get_gripper_state(1))
        self.session.save_arm_pose(step, self.world.get_frame_list())
        return [RobotSpeech.STEP_RECORDED, GazeGoal.NOD]

    def _get_arm_states(self):
        '''Returns the current arms states in the right format'''
        abs_ee_poses = [Arms.get_ee_state(0),
                      Arms.get_ee_state(1)]
        joint_poses = [Arms.get_joint_state(0),
                      Arms.get_joint_state(1)]

        states = [None, None]

        for arm_index in [0, 1]:
            nearest_obj = self.world.get_nearest_object(
                                                abs_ee_poses[arm_index])

            if (nearest_obj == None):
                states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                    abs_ee_poses[arm_index],
                                    joint_poses[arm_index], Object())
            else:
                # Relative
                rel_ee_pose = World.transform(
                                    abs_ee_poses[arm_index],
                                    'base_link', nearest_obj.name)
                states[arm_index] = ArmState(ArmState.OBJECT,
                                    rel_ee_pose,
                                    joint_poses[arm_index], nearest_obj)
        return states

    def execute_action(self, dummy=None):
        '''Starts the execution of the current action'''
	execution_z_offset = -0.00
        if (self.session.n_actions() > 0):
            if (self.session.n_frames() > 0):
                self.session.save_current_action()
                action = self.session.get_current_action()

                if (action.is_object_required()):
                    if (self.world.update_object_pose()):
                        self.session.get_current_action().update_objects(
                                                self.world.get_frame_list())
                        self.arms.start_execution(action, execution_z_offset)
                    else:
                        return [RobotSpeech.OBJECT_NOT_DETECTED,
                                GazeGoal.SHAKE]
                else:
                    self.arms.start_execution(action, execution_z_offset)

                return [RobotSpeech.START_EXECUTION + ' ' +
                        str(self.session.current_action_index), None]
            else:
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def speech_command_cb(self, command):
        '''Callback for when a speech command is received'''
        if command.command in self.responses.keys():
            rospy.loginfo('\033[32m Calling response for command ' +
                          command.command + '\033[0m')
            response = self.responses[command.command]

            if (not self.arms.is_executing()):
                response.respond()
            else:
                if command.command == Command.STOP_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during execution: '
                                  + command.command)
        else:
            switch_command = 'SWITCH_TO_ACTION'
            if (switch_command in command.command):
                action_no = command.command[
                                len(switch_command):len(command.command)]
                action_no = int(action_no)
                if (self.session.n_actions() > 0):
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                         GazeGoal.NOD])
                else:
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
            else:
                rospy.logwarn('\033[32m This command (' + command.command
                              + ') is unknown. \033[0m')

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''
        if (not self.arms.is_executing()):
            if (self.session.n_actions() > 0):
                if (command.command == GuiCommand.SWITCH_TO_ACTION):
                    action_no = command.param
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                         GazeGoal.NOD])
                    response.respond()
                elif (command.command == GuiCommand.SELECT_ACTION_STEP):
                    step_no = command.param
                    self.session.select_action_step(step_no)
                    rospy.loginfo('Selected action step ' + str(step_no))
                else:
                    rospy.logwarn('\033[32m This command (' + command.command
                                  + ') is unknown. \033[0m')
            else:
                response = Response(Interaction.empty_response,
                    [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
        else:
            rospy.logwarn('Ignoring GUI command during execution: ' +
                                command.command)

    def update(self):
        '''General update for the main loop'''
        self.arms.update()

        if (self.arms.status != ExecutionStatus.NOT_EXECUTING):
            if (self.arms.status != ExecutionStatus.EXECUTING):
                self._end_execution()
        if (Interaction._is_recording_motion):
            self._save_arm_to_trajectory()

        is_world_changed = self.world.update()
        if (self.session.n_actions() > 0):
            action = self.session.get_current_action()
            action.update_viz()
            r_target = action.get_requested_targets(0)
            if (r_target != None):
                self.arms.start_move_to_pose(r_target, 0)
                action.reset_targets(0)
            l_target = action.get_requested_targets(1)
            if (l_target != None):
                self.arms.start_move_to_pose(l_target, 1)
                action.reset_targets(1)

            action.delete_requested_steps()

            states = self._get_arm_states()
            action.change_requested_steps(states[0], states[1])

            if (is_world_changed):
                rospy.loginfo('The world has changed.')
                self.session.get_current_action().update_objects(
                                        self.world.get_frame_list())

        time.sleep(0.1)

    def _end_execution(self):
        '''Responses for when the action execution ends'''
        if (self.arms.status == ExecutionStatus.SUCCEEDED):
            Response.say(RobotSpeech.EXECUTION_ENDED)
            Response.perform_gaze_action(GazeGoal.NOD)
        elif (self.arms.status == ExecutionStatus.PREEMPTED):
            Response.say(RobotSpeech.EXECUTION_PREEMPTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        else:
            Response.say(RobotSpeech.EXECUTION_ERROR_NOIK)
            Response.perform_gaze_action(GazeGoal.SHAKE)

        self.arms.status = ExecutionStatus.NOT_EXECUTING

    def record_object_pose(self, dummy=None):
        '''Makes the robot look for a table and objects'''
        if (self.world.update_object_pose()):
            if (self.session.n_actions() > 0):
                self.session.get_current_action().update_objects(
                                            self.world.get_frame_list())
            return [RobotSpeech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]

    def save_experiment_state(self):
        '''Saves session state'''
        self.session.save_current_action()

    @staticmethod
    def empty_response(responses):
        '''Default response to speech commands'''
        return responses
