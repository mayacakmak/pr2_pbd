'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest("pr2_controllers_msgs")

# Generic libraries
import matplotlib.pyplot as plt

import rospy
import time
import numpy
from visualization_msgs.msg import MarkerArray
# Local stuff
from World import World
from RobotSpeech import RobotSpeech
from Session import Session
from Response import Response
from Arms import Arms
from Arm import ArmMode
from actionlib import SimpleActionClient
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import ActionStep, ArmTarget, Object
from pr2_pbd_interaction.msg import GripperAction, ArmTrajectory
from pr2_pbd_interaction.msg import ExecutionStatus, GuiCommand
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal
from pr2_controllers_msgs.msg import SingleJointPositionAction
from pr2_controllers_msgs.msg import SingleJointPositionGoal


class DemoState:
    READY_TO_TAKE = 'READY_TO_TAKE'
    READY_FOR_DEMO = 'READY_FOR_DEMO'
    TOOL_NOT_RECOGNIZED = 'TOOL_NOT_RECOGNIZED'
    HAS_TOOL_NO_SURFACE = 'HAS_TOOL_NO_SURFACE'
    NO_TOOL_NO_SURFACE = 'NO_TOOL_NO_SURFACE'
    RECORDING_DEMO = 'RECORDING_DEMO'
    PLAYING_DEMO = 'PLAYING_DEMO'
    HAS_RECORDED_DEMO = 'HAS_RECORDED_DEMO'

class Interaction:
    '''Finite state machine for the human interaction'''

    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        self.arms = Arms()
        self.world = World()
        self.session = Session(object_list=self.world.get_frame_list(),
                               is_debug=True)
        self._viz_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
        self._demo_state = None
        self._is_busy = True
        self.action_exists = False

        self.torso_client = SimpleActionClient('torso_controller/position_joint_action',
                                                          SingleJointPositionAction)
        rospy.loginfo('Will set up the torso position.')
        self.torso_client.wait_for_server()
        torso_up = .195
        torso_down = .02
        self.torso_client.send_goal(SingleJointPositionGoal(position = (torso_up + torso_down)/2))
        self.torso_client.wait_for_result()

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self.responses = {
            Command.TEST_MICROPHONE: Response(self.default_response,
                                [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD]),
            Command.TAKE_TOOL: Response(self.take_tool, 0),
            Command.RELEASE_TOOL: Response(self.release_tool, 0),
            Command.START_RECORDING: Response(self.start_recording, None),
            Command.STOP_RECORDING: Response(self.stop_recording, None),
            Command.REPLAY_DEMONSTRATION: Response(self.replay_demonstration, None),
            Command.DETECT_SURFACE: Response(self.detect_surface, None)
        }

        rospy.loginfo('Will wait until arms ready to respond.')
        while ((self.arms.get_ee_state(0) is None) or
            (self.arms.get_ee_state(1) is None)):
            time.sleep(0.1)

        rospy.loginfo('Starting to move to the initial pose.')
        # TODO: Make it possible to take with either hand
        self._move_to_arm_pose('take', 0)
        self._move_to_arm_pose('away', 1)
        self._wait_for_arms()
        self._demo_state = DemoState.READY_TO_TAKE
        Response.say(RobotSpeech.HAND_TOOL_REQUEST)
        Response.perform_gaze_action(GazeGoal.GLANCE_RIGHT_EE)
        self._is_busy = False
        rospy.loginfo('Interaction initialized.')

    def take_tool(self, arm_index):
        '''Robot's response to TAKE_TOOL'''
        self._is_busy = True
        self.action_exists = False
        if self._demo_state == DemoState.READY_TO_TAKE:
            ## Robot closes the hand
            Arms.set_gripper_state(arm_index, GripperState.CLOSED, wait=True)
            Response.perform_gaze_action(GazeGoal.LOOK_FORWARD)
            ## Robot moves the hand near the camera to take a look at the tool
            self._move_to_arm_pose('look', arm_index, wait=True)
            time.sleep(1.0)
            self.tool_id = self.world.get_tool_id()

            if self.tool_id is None:
                ## Robot moves the arm back to the person can take the tool
                Response.say(RobotSpeech.ERROR_TOOL_NOT_RECOGNIZED)
                self._move_to_arm_pose('take', 0, wait=True)
                Response.perform_gaze_action(GazeGoal.SHAKE)
                time.sleep(1.0)
                self.arms.set_gripper_state(0, GripperState.OPEN, wait=True)
                self._demo_state = DemoState.READY_TO_TAKE
            else:
                self.action_exists = self.session.new_action(self.tool_id, self.world.get_frame_list())
                if (self.action_exists):
                    Response.say(RobotSpeech.RECOGNIZED_TOOL + str(self.tool_id) 
                                    + '. A demonstration already exists for tool ' + str(self.tool_id))
                else:
                    Response.say(RobotSpeech.RECOGNIZED_TOOL + str(self.tool_id))

                self._demo_state = DemoState.HAS_TOOL_NO_SURFACE
                self.detect_surface()
        else:
            Response.say(RobotSpeech.ERROR_NOT_IN_TAKE_STATE)

        self.log_current_state()
        self._is_busy = False

    def log_current_state(self):
        self.session.update_interaction_state(self._demo_state)
        rospy.loginfo('Current state: ' + self._demo_state)

    def detect_surface(self, dummy=None):
        self._is_busy = True
        if (self._demo_state == DemoState.HAS_TOOL_NO_SURFACE or
            self._demo_state == DemoState.READY_FOR_DEMO or
            self._demo_state == DemoState.HAS_RECORDED_DEMO):

            Response.perform_gaze_action(GazeGoal.LOOK_DOWN)
            self._move_to_arm_pose('away', 0, wait=True)
            time.sleep(2.0)

            ## Robot moves the arm away and looks at the surface
            self.surface = self.world.get_surface()

            if self.surface is None:
                Response.say(RobotSpeech.ERROR_NO_SURFACE)
                Response.perform_gaze_action(GazeGoal.SHAKE)
            else:
                Response.say(RobotSpeech.SURFACE_DETECTED)
                self._move_to_arm_pose('ready', 0, wait=True)
                if self.action_exists:
                    Response.say('Ready for a new demonstration or replay of existing demonstration.')
                    self._demo_state = DemoState.HAS_RECORDED_DEMO
                else:
                    Response.say(RobotSpeech.READY_FOR_DEMO)
                    self._demo_state = DemoState.READY_FOR_DEMO

        self._is_busy = False

    def release_tool(self, arm_index):
        self.busy = True
        if (self._demo_state != DemoState.READY_TO_TAKE and 
            self._demo_state != DemoState.PLAYING_DEMO and
            self._demo_state != DemoState.RECORDING_DEMO):
            self._move_to_arm_pose('take', 0, wait=True)
            self.arms.set_gripper_state(arm_index, GripperState.OPEN, wait=True)
            Response.say(RobotSpeech.TOOL_RELEASED)
            Response.perform_gaze_action(GazeGoal.GLANCE_RIGHT_EE)
            self._demo_state = DemoState.READY_TO_TAKE
        else:
            Response.perform_gaze_action(GazeGoal.SHAKE)
            Response.say(RobotSpeech.ERROR_NOT_IN_RELEASE_STATE)

        self.log_current_state()
        self.busy = False

    def start_recording(self, dummy=None):
        '''Starts recording continuous motion'''
        self.busy = True
        if (self._demo_state == DemoState.READY_FOR_DEMO or
            self._demo_state == DemoState.HAS_RECORDED_DEMO):

            if not self.session.is_current_tool(self.tool_id):
                Response.say('Switching to tool ' + str(self.tool_id) + ' first.')
                time.sleep(0.5)
                self.session.new_action(self.tool_id, self.world.get_frame_list())

            Interaction._arm_trajectory = ArmTrajectory()
            Interaction._trajectory_start_time = rospy.Time.now()

            if self.session.n_frames() > 0:
                self.session.clear_current_action()

            self.relax_arm(0)
            self._demo_state = DemoState.RECORDING_DEMO
            Response.say(RobotSpeech.STARTED_RECORDING)
            Response.perform_gaze_action(GazeGoal.NOD)
        elif (self._demo_state == DemoState.RECORDING_DEMO):
            Response.perform_gaze_action(GazeGoal.SHAKE)
            Response.say(RobotSpeech.ERROR_ALREADY_RECORDING)
        else:
            Response.perform_gaze_action(GazeGoal.SHAKE)
            Response.say(RobotSpeech.ERROR_NOT_READY_TO_RECORD)

        self.log_current_state()
        self.busy = False


    def _cluster_demonstration(self, arm_trajectory):
        ''' Find meaningful clusters in the trajectory'''

        clusters = []
        
        n_points = len(arm_trajectory.timing)
        r_traj = arm_trajectory.rArm[:]

        # First determine the lowest point in the trajectory
        all_x = []
        all_y = []
        all_z = []
        for i in range(n_points):
            point_x = r_traj[i].ee_pose.position.x
            point_y = r_traj[i].ee_pose.position.y
            point_z = r_traj[i].ee_pose.position.z
            all_x.append(point_x)
            all_y.append(point_y)
            all_z.append(point_z)
            clusters.append(-1) #unassigned    

        min_z = min(all_z)

        ################################################
        ######## LETS PLOT STUFF TO GET A BETTER IDEA

        num_bins = 50
        # the histogram of the data
        plt.subplot(4, 1, 1)
        plt.plot(range(n_points), all_x, 'ro-')
        plt.ylabel('x')

        plt.subplot(4, 1, 2)
        plt.plot(range(n_points), all_y, 'bo-')
        plt.ylabel('y')

        plt.subplot(4, 1, 3)
        plt.plot(range(n_points), all_z, 'go-')
        plt.ylabel('z')

        plt.subplot(4, 1, 4)
        n, bins, patches = plt.hist(all_z, num_bins, normed=1, facecolor='yellow', alpha=0.5)
        plt.xlabel('z (histogram bins)')
        plt.ylabel('occurance')

        # Tweak spacing to prevent clipping of ylabel
        plt.subplots_adjust(left=0.15)
        plt.show()

        ################################################

        # Assign points close to lowest point as application (cluster 2)
        for i in range(n_points):
            point_z = r_traj[i].ee_pose.position.z
            if (numpy.abs(point_z - min_z) < 0.04):
                clusters[i] = ArmTrajectory.APPLICATION

        # Assign points at the beginning as entry
        index = 0
        while (clusters[index] != ArmTrajectory.APPLICATION):
            clusters[index] = ArmTrajectory.START
            index = index + 1

        # Assign points at the end beginning as exit
        index = n_points - 1
        while (clusters[index] != ArmTrajectory.APPLICATION):
            clusters[index] = ArmTrajectory.END
            index = index -1

        # Assign mid points based on their diff and z from lowest point
        for i in range(n_points-1):
            point_z = r_traj[i].ee_pose.position.z
            if (numpy.abs(point_z - min_z) < 0.12):
                if clusters[i] != ArmTrajectory.APPLICATION:

                    next_point_z = r_traj[i+1].ee_pose.position.z
                    diff_z = next_point_z - point_z

                    if (diff_z) >= 0:
                        clusters[i] = ArmTrajectory.EXIT
                    else:
                        clusters[i] = ArmTrajectory.ENTRY

        #Everything else is connectors
        for i in range(n_points-1):
            if clusters[i] == -1:
                clusters[i] = ArmTrajectory.CONNECTOR

        # Finally do some smoothing
        for i in range(n_points-2):
            c1 = clusters[i]
            c2 = clusters[i+1]
            c3 = clusters[i+2]

            if c1==c3 and c1!=c2:
                clusters[i+1] = c1
        
        return clusters

    def compute_clusters(self):
        action = self.session.get_current_action()
        if action is not None:
            arm_trajectory = action.get_trajectory()
            clusters = self._cluster_demonstration(arm_trajectory)
            action.update_trajectory(clusters)

    def compute_trajectory(self):
        #TODO
        pass

    def plot_trajectory(self):

        action = self.session.get_current_action()
        if action is not None:
            arm_trajectory = action.get_trajectory()
                
           
        n_points = len(arm_trajectory.timing)
        r_traj = arm_trajectory.rArm[:]

        # First determine the lowest point in the trajectory
        all_x = []
        all_y = []
        all_z = []
        for i in range(n_points):
            point_x = r_traj[i].ee_pose.position.x
            point_y = r_traj[i].ee_pose.position.y
            point_z = r_traj[i].ee_pose.position.z
            all_x.append(point_x)
            all_y.append(point_y)
            all_z.append(point_z)
      


        ################################################
        ######## LETS PLOT STUFF TO GET A BETTER IDEA

        num_bins = 50
        # the histogram of the data
        plt.subplot(4, 1, 1)
        plt.plot(range(n_points), all_x, 'ro-')
        plt.ylabel('x')

        plt.subplot(4, 1, 2)
        plt.plot(range(n_points), all_y, 'bo-')
        plt.ylabel('y')

        plt.subplot(4, 1, 3)
        plt.plot(range(n_points), all_z, 'go-')
        plt.ylabel('z')

        plt.subplot(4, 1, 4)
        n, bins, patches = plt.hist(all_z, num_bins, normed=1, facecolor='yellow', alpha=0.5)
        plt.xlabel('z (histogram bins)')
        plt.ylabel('occurance')

        # Tweak spacing to prevent clipping of ylabel
        plt.subplots_adjust(left=0.15)
        plt.show()


    def stop_recording(self, dummy=None):
        '''Stops recording continuous motion'''
        self.busy = True

        if (self._demo_state == DemoState.RECORDING_DEMO):
            
            traj_step = ActionStep()
            traj_step.type = ActionStep.ARM_TRAJECTORY
            waited_time = Interaction._arm_trajectory.timing[0]
            n_points = len(Interaction._arm_trajectory.timing)
            for i in range(n_points):
                Interaction._arm_trajectory.timing[i] -= waited_time
                Interaction._arm_trajectory.timing[i] += rospy.Duration(0.1)
            
            self._demo_state = DemoState.HAS_RECORDED_DEMO

            '''If motion was relative, record transformed pose'''
            traj_step.armTrajectory = ArmTrajectory(
                self.surface,
                Interaction._arm_trajectory.rArm[:],
                Interaction._arm_trajectory.lArm[:],
                Interaction._arm_trajectory.timing[:],
                Interaction._arm_trajectory.rRefFrame,
                Interaction._arm_trajectory.lRefFrame,
                Interaction._arm_trajectory.rRefFrameObject,
                Interaction._arm_trajectory.lRefFrameObject,
                [],
                [],
                []
            )
            
            traj_step.gripperAction = GripperAction(
                                        self.arms.get_gripper_state(0),
                                        self.arms.get_gripper_state(1))
                                        
            self.session.add_step_to_action(traj_step,
                                        self.world.get_frame_list())
            

            Interaction._arm_trajectory = None
            Interaction._trajectory_start_time = None
            self.session.save_current_action()
            self.freeze_arm(0)
            
            Response.say(RobotSpeech.STOPPED_RECORDING)
            Response.perform_gaze_action(GazeGoal.NOD)
            time.sleep(1.5)
            self._move_to_arm_pose('ready', 0, wait=True)

        else:

            Response.say(RobotSpeech.ERROR_NOT_RECORDING)
            Response.perform_gaze_action(GazeGoal.SHAKE)

        self.log_current_state()
        self.busy = False

    def replay_demonstration(self, dummy=None):
        '''Starts the execution of the current demonstration'''
        self.busy = True
        execution_z_offset = 0.00
        if (self._demo_state == DemoState.HAS_RECORDED_DEMO):
            
            if self.session.current_action_has_demo():
                if not self.session.is_current_tool(self.tool_id):
                    Response.say('Warning. The demonstration does not match the current tool.')
                    time.sleep(1.5)

                self.session.save_current_action()
                action = self.session.get_current_action()
                self.arms.start_execution(action, execution_z_offset)
                Response.say(RobotSpeech.STARTED_REPLAY)
                self._demo_state = DemoState.PLAYING_DEMO
            else:
                Response.say('A demonstration has not been recorded yet.')
                Response.perform_gaze_action(GazeGoal.SHAKE)
                self._demo_state = DemoState.READY_FOR_DEMO
        else:
            Response.say(RobotSpeech.ERROR_CANNOT_REPLAY)
            Response.perform_gaze_action(GazeGoal.SHAKE)

        self.log_current_state()
        self.busy = False

    def _end_replay(self):
        '''Responses for when the action execution ends'''
        if (self.arms.status == ExecutionStatus.SUCCEEDED):
            if self._demo_state == DemoState.PLAYING_DEMO:
                Response.say(RobotSpeech.ENDED_REPLAY)
                Response.perform_gaze_action(GazeGoal.NOD)
                self._demo_state = DemoState.HAS_RECORDED_DEMO
            else:
                rospy.loginfo('Non-replay motion successful.')
        elif (self.arms.status == ExecutionStatus.PREEMPTED):
            if self._demo_state == DemoState.PLAYING_DEMO:
                Response.say(RobotSpeech.ERROR_PREEMPTED_REPLAY)
                Response.perform_gaze_action(GazeGoal.SHAKE)
                self._demo_state = DemoState.HAS_RECORDED_DEMO
            else:
                rospy.loginfo('Non-replay motion pre-empted.')
        else:
            if self._demo_state == DemoState.PLAYING_DEMO:
                Response.say(RobotSpeech.ERROR_REPLAY_NOT_POSSIBLE)
                Response.perform_gaze_action(GazeGoal.SHAKE)
                self._demo_state = DemoState.HAS_RECORDED_DEMO
            else:
                rospy.loginfo('Non-replay motion has no IK.')

        self.arms.status = ExecutionStatus.NOT_EXECUTING

    def default_response(self, responses):
        '''Default response to speech commands'''
        self.busy = True
        speech_resp = responses[0]
        gaze_resp = responses[1]

        # Speech response
        if (speech_resp != None):
            Response.say(speech_resp)
        
        # Gaze response
        if (gaze_resp != None):
            Response.perform_gaze_action(gaze_resp)

        self.log_current_state()
        self.busy = False

    def relax_arm(self, arm_index):
        '''Relaxes arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.RELEASE):
            return True
        else:
            return False

    def freeze_arm(self, arm_index):
        '''Stiffens arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.HOLD):
            return True
        else:
            return False

    def stop_execution(self, dummy=None):
        '''Stops ongoing execution'''
        if (self._demo_state == DemoState.PLAYING_DEMO and
            self.arms.is_executing()):
            self.arms.stop_execution()
            
            Respoonse.say(RobotSpeech.STOPPING_EXECUTION)
            Response.perform_gaze_action(GazeGoal.NOD)
        else:
            Response.perform_gaze_action(GazeGoal.SHAKE)
            Respoonse.say(RobotSpeech.ERROR_IS_NOT_PLAYING)

    def _save_arm_to_trajectory(self):
        '''Saves current arm state into continuous trajectory'''
        if (Interaction._arm_trajectory != None):
            states =  self._get_arm_states()     
            Interaction._arm_trajectory.rArm.append(states[0])
            Interaction._arm_trajectory.lArm.append(states[1])
            Interaction._arm_trajectory.timing.append(
                        rospy.Time.now() - Interaction._trajectory_start_time)

    def _move_to_arm_pose(self, pose_name, arm_index, wait=False):
        '''Moves the robot's arm to a pre-specified arm pose'''
        action = self.session.pose_set[pose_name]
        if action is None:
            rospy.logwarn('Arm pose does not exist:' + pose_name)
        else:
            time.sleep(0.5)
            step = action.get_step(0)
            if arm_index == 0:
                self.arms.start_move_to_pose(step.armTarget.rArm, 0)
                if wait:
                    self._wait_for_arms()
                #self.arms.set_gripper_state(0, step.gripperAction.rGripper, wait=wait)
            else:
                self.arms.start_move_to_pose(step.armTarget.lArm, 1)
                if wait:
                    self._wait_for_arms()
                #self.arms.set_gripper_state(1, step.gripperAction.lGripper, wait=wait)

            rospy.loginfo('Moved arm ' + str(arm_index) + ' to pose ' + pose_name)

    def _wait_for_arms(self):
        rospy.loginfo('Will wait until the arms get in place.')
        time.sleep(0.5)
        while (self.arms.is_executing()):
            time.sleep(0.1)
        rospy.loginfo('Arms are in place.')

    def _get_arm_states(self):
        '''Returns the current arms states in the right format'''
        abs_ee_poses = [Arms.get_ee_state(0),
                      Arms.get_ee_state(1)]
        joint_poses = [Arms.get_joint_state(0),
                      Arms.get_joint_state(1)]

        states = [None, None]

        for arm_index in [0, 1]:
            states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                    abs_ee_poses[arm_index],
                                    joint_poses[arm_index], Object())
        return states

    def speech_command_cb(self, command):
        '''Callback for when a speech command is received'''
        if command.command in self.responses.keys():
            rospy.loginfo('\033[32m Calling response for command ' +
                          command.command + '\033[0m')
            response = self.responses[command.command]

            if (not self.arms.is_executing() and not self._is_busy):
                response.respond()
            else:
                if command.command == Command.STOP_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during execution or busy: '
                                  + command.command)
        else:
            rospy.logwarn('\033[32m This command (' + command.command
                              + ') is unknown. \033[0m')

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''
        if (not self.arms.is_executing()):
            if ((self._demo_state != DemoState.RECORDING_DEMO) and
                (self._demo_state != DemoState.PLAYING_DEMO)):
                if (command.command == GuiCommand.SWITCH_TO_ACTION):
                    action_no = command.param
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    tool_name = self.session.get_current_action().name
                    response = Response(self.default_response,
                        [RobotSpeech.SWITCH_SKILL + tool_name,
                         GazeGoal.NOD])
                    response.respond()
                elif (command.command == GuiCommand.COMPUTE_CLUSTERS):
                    self.compute_clusters()
                    rospy.loginfo('Computing clusters.')
                elif (command.command == GuiCommand.COMPUTE_TRAJECTORY):
                    self.compute_trajectory()
                    rospy.loginfo('Computing trajectory.')
                elif (command.command == GuiCommand.PLOT_TRAJECTORY):
                    self.plot_trajectory()
                    rospy.loginfo('Ploting trajectory.')
                elif (command.command == GuiCommand.SELECT_ACTION_STEP):
                    step_no = command.param
                    self.session.select_action_step(step_no)
                    rospy.loginfo('Selected action step ' + str(step_no))
                elif (command.command == GuiCommand.NEW_EXPERIMENT):
                    self.session.create_new_experiment()
                    Response.say('Created experiment ' + str(self.session.current_experiment))
                    Response.perform_gaze_action(GazeGoal.NOD)
                elif (command.command == GuiCommand.SWITCH_TO_EXPERIMENT):
                    exp_no = command.param
                    self.session.switch_to_experiment(exp_no,
                                                  self.world.get_frame_list())
                    response = Response(self.default_response,
                        [RobotSpeech.SWITCH_EXPERIMENT + str(exp_no),
                         GazeGoal.NOD])
                    response.respond()
                else:
                    rospy.logwarn('\033[32m This command (' + command.command
                                  + ') is unknown. \033[0m')
            else:
                rospy.logwarn('Ignoring GUI command during execution/recording states: ' +
                                    command.command)
        else:
            rospy.logwarn('Ignoring GUI command during execution: ' +
                                command.command)

    def save_experiment_state(self):
        pass

    def update(self):
        '''General update for the main loop'''
        self.arms.update()
        self.world.update()

        if (self.arms.status != ExecutionStatus.NOT_EXECUTING):
            if (self.arms.status != ExecutionStatus.EXECUTING):
                self._end_replay()

        if (self._demo_state == DemoState.RECORDING_DEMO):
            self._save_arm_to_trajectory()

        if (self.session.n_actions() > 0):
            action = self.session.get_current_action()
            action.update_viz()

        time.sleep(0.01)