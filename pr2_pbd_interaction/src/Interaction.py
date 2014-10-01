'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest("pr2_controllers_msgs")

# Generic libraries
import matplotlib.pyplot as plt
import scipy.signal as sgnl

import rospy
import time
import numpy
import math
from visualization_msgs.msg import MarkerArray
# Local stuff
from World import World
from RobotSpeech import RobotSpeech
from Session import Session
from Response import Response
from Arms import Arms
from Arm import ArmMode
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose
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
    X_DIR = 0
    Y_DIR = 1
    ERROR_NEGATIVE_SLOPES = 2
    ERROR_INDETERMINATE_DIR = 3

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
	torso_pos = torso_down + (torso_up-torso_down)*0.7
        self.torso_client.send_goal(SingleJointPositionGoal(position = torso_pos))
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
        action = self.session.get_current_action()
        if action is not None:
            arm_trajectory = action.get_trajectory()

        #recorded table surface:
        # self.surface = self.world.get_surface()  
        # arm_trajectory.table_corners = self.surface  
        table_corner = arm_trajectory.table_corners[:]

        #ToDo: add the size of the old table with parameter like H and W

        rospy.loginfo('table corner 1 ' + str(table_corner[0].position.x) + ' ' + str(table_corner[0].position.y))
        rospy.loginfo('table corner 2 ' + str(table_corner[1].position.x) + ' ' +str(table_corner[1].position.y))
        rospy.loginfo('table corner 3 ' + str(table_corner[2].position.x) + ' ' +str(table_corner[2].position.y))
        rospy.loginfo('table corner 4 ' + str(table_corner[3].position.x) + ' ' +str(table_corner[3].position.y))
        rospy.loginfo('the size of the demonstrated trajecotry is ' + str(len(arm_trajectory.timing)))


        # get the new surface:

        self.surface = self.world.get_surface()
        new_table = self.surface #arm_traj_newSurface.table_corners[:]

        rospy.loginfo('new table corner 1 ' + str(new_table[0].position.x) + ' ' + str(new_table[0].position.y))
        rospy.loginfo('new table corner 2 ' + str(new_table[1].position.x) + ' ' + str(new_table[1].position.y))
        rospy.loginfo('new table corner 3 ' + str(new_table[2].position.x) + ' ' + str(new_table[2].position.y))
        rospy.loginfo('new table corner 4 ' + str(new_table[3].position.x) + ' ' + str(new_table[3].position.y))

        # """to re-align the simulated surface with the physical card-board -- solve the mirrored misalignment issue: """


        # temp_table_0_x = table_corner[0].position.x
        # temp_table_0_y = table_corner[0].position.y

        # temp_table_2_x = table_corner[2].position.x
        # temp_table_2_y = table_corner[2].position.y

        # table_corner[0].position.x = table_corner[1].position.x
        # table_corner[0].position.y = table_corner[1].position.y

        # table_corner[2].position.x = table_corner[3].position.x
        # table_corner[2].position.y = table_corner[3].position.y


        # table_corner[1].position.x = temp_table_0_x
        # table_corner[1].position.y = temp_table_0_y

        # table_corner[3].position.x = temp_table_2_x
        # table_corner[3].position.y = temp_table_2_y




        # try to create a new cluster and also visualize it.

        
        clusters = []
        n_points = len(arm_trajectory.timing)
        r_traj = arm_trajectory.rArm[:]

        # First determine the lowest point in the trajectory
        all_x = []
        all_y = []
        all_z = []
        for i in range(n_points):

            #making modification for a new trajectory
            point_x = r_traj[i].ee_pose.position.x
            point_y = r_traj[i].ee_pose.position.y
            point_z = r_traj[i].ee_pose.position.z
            all_x.append(point_x)
            all_y.append(point_y)
            all_z.append(point_z)
            clusters.append(-1) #unassigned    

            #just a test
            # r_traj[i].ee_pose.position.x.append(point_x)
            # r_traj[i].ee_pose.position.y.append(point_y)


        min_z = min(all_z)

        ###################Peak detection#############################

        peak_z = []
        # Assign points close to lowest point as application (cluster 2)
        # for i in range(n_points):
        #     point_z = r_traj[i].ee_pose.position.z
        #     if (numpy.abs(point_z - min_z) < 0.025):
        #         clusters[i] = ArmTrajectory.APPLICATION
        #         peak_z.append(point_z)

        data = all_z

        '''filtering'''
        # window = sgnl.general_gaussian(51, p=0.5, sig=20)
        # filtered = sgnl.fftconvolve(window, data)
        # filtered = (numpy.average(data) / numpy.average(filtered)) * filtered
        # filtered = numpy.roll(filtered, -25)
        ''' method 1 '''
        peakind = sgnl.find_peaks_cwt(data, numpy.arange(1,250), noise_perc=15)
        ''' method 2 '''
        # peakind = sgnl.find_peaks_cwt(filtered, numpy.arange(100,200))
        ''' method 3: finding the maximum values from filtered curves: '''
        # peakind = sgnl.argrelmax(filtered, order=5)

        ''' this is for converting tuple peakind to an array: '''
        # peak_index = []
        # for item in peakind:
        #     peak_index.extend(item)

        # peakind = peakind_ini[1:len(peakind_ini)-1]

        # print peakind
        peak_data = []
        for i in peakind:
            peak_data.append(data[i])



        plt.plot(data, linewidth=10)
        # plt.plot(filtered)
        # plt.plot(peak_index, filtered[peakind], 'ro')
        plt.plot(peakind, peak_data, 'ro', markersize=30)
        plt.xlabel('Time stamps(-)',fontsize=40)
        plt.ylabel('End-effector vertcial position (m)', fontsize=50)
        plt.xlim(0, len(all_z))
        plt.tick_params(axis='x', labelsize=40)
        plt.tick_params(axis='y', labelsize=40)
        plt.show()

        
        """
        Using sliding window method to find clusters
        Starts at peaks and computes variance of points within the window
        Window moves until the variance is within a tolerance
        Sections with small variance are considered part of the application
        """

        window_size = 50
        tolerance = 0.000015

        peak_index = peakind

        app_cluster_bounds = []

        for i in range(len(peak_index) - 1):

            current_app_cluster = []

            current_ind = [peak_index[i], peak_index[i+1]]
            z_segment = all_z[current_ind[0]:current_ind[1]]
            
            sliding_window = []
            _j = 0
            for j in range(len(z_segment)):
                _j = j + 1
                sliding_window = z_segment[j:j+window_size]
                variance = numpy.var(sliding_window)
                if (variance < tolerance):
                    break

            transition_entry = z_segment[:j]

            _k = 0
            for k in range(len(z_segment)):
                _k = _k + 1
                sliding_window = z_segment[len(z_segment) - k - window_size:len(z_segment) - k]
                variance = numpy.var(sliding_window)
                if (variance < tolerance):
                    break

            transition_exit = z_segment[_k:]

            application_cluster = z_segment[_j:_k]

            rospy.loginfo('Size clusters: ' + str(len(clusters)))
            clusters[(_j + peak_index[i]):(peak_index[i + 1] - _k)] = [ArmTrajectory.APPLICATION] * numpy.absolute(_j + peak_index[i] - (peak_index[i + 1] - _k))

            if (_j < 5):
                clusters[peak_index[i]: (_j + peak_index[i])] = [ArmTrajectory.APPLICATION] * _j
                _j = 0
            else:
                clusters[peak_index[i]: (_j + peak_index[i])] = [ArmTrajectory.ENTRY] * _j


            if (_k < 5):
                clusters[(peak_index[i + 1] - _k):peak_index[i+1]] = [ArmTrajectory.APPLICATION] * _k
                _k = 0
            else:
                clusters[(peak_index[i + 1] - _k):peak_index[i+1]] = [ArmTrajectory.EXIT] * _k

    
            current_app_cluster = range(peak_index[i] + _j, peak_index[i + 1] - _k)
            app_cluster_bounds.append(current_app_cluster)

        rospy.loginfo('First 10 of app_cluster_bounds: ' + str(app_cluster_bounds[0][:10]))
        clusters[:5] = [-1]*5
        clusters[(len(clusters) - 5):] = [-1]*5 

    
        """
        We remove application segments that have
        average heights much greater than the median.
        """

        outlier_heights = []
        outlier_length_x = []
        outlier_length_y = []
        
        slopes_x_pos = 0
        slopes_x_neg = 0
        slopes_y_pos = 0
        slopes_y_neg = 0
        for cluster in app_cluster_bounds:
            if not cluster:
                continue
            x_app = []
            y_app = []
            heights = []
            lengths_x = numpy.absolute(all_x[cluster[0]] - all_x[cluster[len(cluster) -1]])
            lengths_y = numpy.absolute(all_y[cluster[0]] - all_y[cluster[len(cluster) -1]])
            for ind in cluster:
                heights.append(all_z[ind])
                x_app.append(all_x[ind])
                y_app.append(all_y[ind])
            n = len(x_app)
            t = list(xrange(n))
            slope_x = numpy.polyfit(t, x_app,1)[0]
            slope_y = numpy.polyfit(t, y_app,1)[0]
           
            if (numpy.abs(slope_x) > numpy.abs(slope_y)):
                if (slope_x > 0):
                    slopes_x_pos = slopes_x_pos + 1
                else:
                    slopes_x_neg = slopes_x_neg + 1
            else:
                if (slope_y > 0):
                    slopes_y_pos = slopes_y_pos + 1
                else:
                    slopes_y_neg = slopes_y_neg + 1

            mean = numpy.mean(heights)
            outlier_length_x.append(lengths_x)
            outlier_length_y.append(lengths_y)
            outlier_heights.append(mean)


        outlier_height_indices = self.find_outliers(outlier_heights, 2, True)


        #Commented out the part that looks for outliers based on x and y

        #outlier_length_y_indices = self.find_outliers(outlier_length_y)
        #outlier_length_x_indices = self.find_outliers(outlier_length_x)
        #mergedlist = list(set(outlier_height_indices + outlier_length_x_indices + outlier_length_y_indices))

        mergedlist = outlier_height_indices

        outliers = []

        for i in mergedlist:
            a = app_cluster_bounds[i]
            for j in a:
                outliers.append(j)

        for i in outliers:
            clusters[i] = -1

       
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

        action.update_trajectory(clusters)


        """
        Found slopes over the various segments in the 
        application and repetition directions.
        The direction with the greatest absolute slope over
        the segment is the application direction.
        """

        slopes = [slopes_x_pos, slopes_x_neg, slopes_y_pos, slopes_y_neg]
        
        app_direction = max(slopes)

        instances = 0
        for i in slopes:
            if (i == app_direction):
                instances = instances + 1

        if (instances > 1):
            rospy.loginfo('Indeterminate direction')
            return

        x_pos = False
        if ((app_direction == slopes_x_pos) or (app_direction == slopes_x_neg)):  
            if (slopes_x_pos > slopes_x_neg):
                x_pos = True
            elif (slopes_x_pos == slopes_x_neg):
                rospy.loginfo('Indeterminate x direction')
                return

        y_pos = False
        if ((app_direction == slopes_y_pos) or (app_direction == slopes_y_neg)):
            if (slopes_y_pos > slopes_y_neg):
                y_pos = True
            elif (slopes_y_pos == slopes_y_neg):
                rospy.loginfo('Indeterminate y direction')
                return


        """"finding the repetition number and starting point of the cleaning action"""

        clusters_rep = []
        num_reps = 0
   
        # First determine the lowest point in the trajectory
        for i in range(n_points):
            clusters_rep.append(-1) #unassigned
 
         # Assign points close to lowest point as application (cluster 2)
        for i in range(n_points):
            point_z = r_traj[i].ee_pose.position.z
            if (numpy.abs(point_z - min_z) < 0.05):
                clusters_rep[i] = ArmTrajectory.APPLICATION
                
        # Assign points at the beginning as entry
        index = 0
        while (clusters[index] != ArmTrajectory.APPLICATION):
            clusters_rep[index] = ArmTrajectory.START
            index = index + 1
        # Assign points at the end beginning as exit
        index = n_points - 1
        while (clusters_rep[index] != ArmTrajectory.APPLICATION):
            clusters_rep[index] = ArmTrajectory.END
            index = index -1
        # Assign mid points based on their diff and z from lowest point
        for i in range(n_points-1):
            point_z = r_traj[i].ee_pose.position.z
            if (numpy.abs(point_z - min_z) < 0.12):
                if clusters_rep[i] != ArmTrajectory.APPLICATION:
                    next_point_z = r_traj[i+1].ee_pose.position.z
                    diff_z = next_point_z - point_z
                    if (diff_z) >= 0:
                        clusters_rep[i] = ArmTrajectory.EXIT
                    else:
                        clusters_rep[i] = ArmTrajectory.ENTRY
        #Everything else is connectors
        for i in range(n_points-1):
            if clusters_rep[i] == -1:
                clusters_rep[i] = ArmTrajectory.CONNECTOR
        # Finally do some smoothing
        for i in range(n_points-2):
            c1 = clusters_rep[i]
            c2 = clusters_rep[i+1]
            c3 = clusters_rep[i+2]
            if c1==c3 and c1!=c2:
                clusters_rep[i+1] = c1

        start_rep = []
        end_rep = []
        means = []

        for i in range(len(clusters_rep)):
            if ((i > 0) and (clusters_rep[i] == ArmTrajectory.APPLICATION) and (clusters_rep[i-1] != ArmTrajectory.APPLICATION)):
                    num_reps = num_reps + 1
                    start_rep.append(i)
                    rospy.loginfo('Number of reps so far: ' + str(num_reps))
            if ((i < (len(clusters_rep) - 1)) and (clusters_rep[i] == ArmTrajectory.APPLICATION) and (clusters_rep[i+1] != ArmTrajectory.APPLICATION)):
                    end_rep.append(i)

        if (len(start_rep) != len(end_rep)):
            rospy.loginfo('Error: Cleaning units have unequal number of starts and ends')

        for i in range(len(start_rep)):
            current_rep = []
            for j in range(start_rep[i], end_rep[i]):
                if ((app_direction == slopes_x_pos) or (app_direction == slopes_x_neg)): 
                    current_rep.append(all_y[j])
                if ((app_direction == slopes_y_pos) or (app_direction == slopes_y_neg)): 
                    current_rep.append(all_x[j])
            rospy.loginfo('Mean of current rep: ' + str(numpy.mean(current_rep)))
            means.append(numpy.mean(current_rep))


        pos_dir = 0
        neg_dir = 0
        dist_between_means = []
        for k in range(len(means) -1):
            dist_between_means.append(means[k + 1] - means[k])
            if (numpy.abs(means[k] - means[k + 1]) < 0.02):
                num_reps = num_reps - 1
            if (means[k] < means[k + 1]):
                pos_dir = pos_dir + 1
            elif (means[k] > means[k + 1]):
                neg_dir = neg_dir + 1

        pos_rep = None 
        if (pos_dir > neg_dir):
            rospy.loginfo('Postive rep dir')
            pos_rep = True
        elif (neg_dir > pos_dir):
            rospy.loginfo('Negative rep dir')
            pos_rep = False
        else:
            rospy.loginfo('Indeterminate repetition direction')
            return

        rospy.loginfo('New number of reps: ' + str(num_reps))

        #Find length of table repetition direction
        corner_dist_rep = 0
        corner_dist_app = 0
        if ((app_direction == slopes_x_pos) or (app_direction == slopes_x_neg)): 
            corner_dist_rep = numpy.abs(new_table[0].position.y - new_table[1].position.y)
            corner_dist_app = numpy.abs(new_table[1].position.x - new_table[3].position.x)   
        elif ((app_direction == slopes_y_pos) or (app_direction == slopes_y_neg)): 
            corner_dist_rep = numpy.abs(new_table[1].position.x - new_table[3].position.x)
            corner_dist_app = numpy.abs(new_table[0].position.y - new_table[1].position.y)

        rep_dist = corner_dist_rep/num_reps
        if (numpy.abs(rep_dist - numpy.mean(dist_between_means)) > 0.03):
            rep_dist =  numpy.mean(dist_between_means)

        rospy.loginfo('Rep dist: ' + str(rep_dist))

        # In format [rep, app]
        prev_start = [all_x[start_rep[0]], all_y[start_rep[0]]]

        #Find repetition direction


        """
        Finding the actual cleaning unit by
        finding the peaks along individual 
        repetition direction segments. 
        """

        cu_peaks = []
        cluster_num = 0
        best_cu = []
        diff = numpy.inf
        first = app_cluster_bounds.pop(0)
        last = app_cluster_bounds.pop(len(app_cluster_bounds) - 1)
        rospy.loginfo('Total repetitions: ' + str(len(app_cluster_bounds)))
        tried_everything = 0

        cu_list = []
        diff_list = []
        var_list_x = []
        var_list_y = []

        while(tried_everything < 2):
            if (app_direction == slopes_x_pos):
                rospy.loginfo('Positive X is App direction')
                for cluster in app_cluster_bounds:
                    rospy.loginfo('cluster num: ' + str(cluster_num))
                    cluster_num = cluster_num + 1
                    if (y_pos):
                        cu_peaks = self.find_cleaning_unit(all_y, all_x, cluster, True, True)
                    else:
                        cu_peaks = self.find_cleaning_unit(all_y, all_x, cluster, False, True)
                    if not cu_peaks:
                        rospy.loginfo('No cleaning peaks in that segment')
                        continue

                    for i in cu_peaks:
                        diff = numpy.abs(all_y[i[0]] - all_y[i[1]])
                        diff_list.append(diff)
                    cu_list = cu_list + cu_peaks

            elif (app_direction == slopes_y_pos):
                rospy.loginfo('Positive Y is App direction')
                for cluster in app_cluster_bounds:
                    rospy.loginfo('cluster num: ' + str(cluster_num))
                    cluster_num = cluster_num + 1
                    if (x_pos):
                        cu_peaks = self.find_cleaning_unit(all_x, all_y, cluster, True, True)
                    else:
                        cu_peaks = self.find_cleaning_unit(all_x, all_y, cluster, False, True)
                    if not cu_peaks:
                        rospy.loginfo('No cleaning peaks in that segment')
                        continue
                    for i in cu_peaks:
                        diff = numpy.abs(all_x[i[0]] - all_x[i[1]])
                        diff_list.append(diff)
                    cu_list = cu_list + cu_peaks            
            elif (app_direction == slopes_x_neg):
                rospy.loginfo('Negative X is App direction')
                for cluster in app_cluster_bounds:
                    rospy.loginfo('cluster num: ' + str(cluster_num))
                    cluster_num = cluster_num + 1
                    if (y_pos):
                        cu_peaks = self.find_cleaning_unit(all_y, all_x, cluster, True, False)
                    else:
                        cu_peaks = self.find_cleaning_unit(all_y, all_x, cluster, False, False)
                    if not cu_peaks:
                        rospy.loginfo('No cleaning peaks in that segment')
                        continue
                    for i in cu_peaks:
                        diff = numpy.abs(all_y[i[0]] - all_y[i[1]])
                        diff_list.append(diff)
                    cu_list = cu_list + cu_peaks                      
            elif (app_direction == slopes_y_neg):
                rospy.loginfo('Negative Y is App direction')
                for cluster in app_cluster_bounds:
                    rospy.loginfo('cluster num: ' + str(cluster_num))
                    cluster_num = cluster_num + 1
                    if (x_pos):
                        cu_peaks = self.find_cleaning_unit(all_x, all_y, cluster, True, False)
                    else:
                        cu_peaks = self.find_cleaning_unit(all_x, all_y, cluster, False, False)
                    if not cu_peaks:
                        rospy.loginfo('No cleaning peaks in that segment')
                        continue
                    for i in cu_peaks:
                        diff = numpy.abs(all_x[i[0]] - all_x[i[1]])
                        diff_list.append(diff)
                    cu_list = cu_list + cu_peaks

            if not cu_list:
                app_cluster_bounds = [first, last]
                tried_everything = tried_everything + 1
            else:
                break

        
        """
        Calculate x and y variances over each cleaning unit
        """

        for i in cu_list:
            current_cu_x = []
            current_cu_y = []
            for j in range(i[0],i[1]):
                current_cu_x.append(all_x[j])
                current_cu_y.append(all_y[j])
            var_list_x.append(numpy.var(current_cu_x))
            var_list_y.append(numpy.var(current_cu_y))

            
        """
        Use variances to reject outliers
        """

        outlier_x_indices = self.find_outliers(var_list_x, 2, False)
        outlier_y_indices = self.find_outliers(var_list_y, 2, False)

        merged = list(set(outlier_x_indices + outlier_y_indices))

        cu_to_remove = []
        diffs_to_remove = []
        var_list_x_remove = []
        var_list_y_remove = []


        for ind in merged:
            cu_to_remove.append(cu_list[ind])
            diffs_to_remove.append(diff_list[ind])
            var_list_x_remove.append(var_list_x[ind])
            var_list_y_remove.append(var_list_y[ind])

        for cu in cu_to_remove:
            cu_list.remove(cu)

        for diff in diffs_to_remove:
            diff_list.remove(diff)

        for var_x in var_list_x_remove:
            var_list_x.remove(var_x)

        for var_y in var_list_y_remove:
            var_list_y.remove(var_y) 


        """
        Pick cleaning unit with closest starting 
        and ending points in repetition direction
        """

        min_val = min(diff_list)
        index = diff_list.index(min_val)
        best_cu = cu_list[index]

        """
        Check if cleaning unit is very flat and truncate
        """

        var_tol = 0.002 #TODO: pick a good variance tolerance to detect flat cleaning units

        if ((app_direction == slopes_x_pos) or (app_direction == slopes_x_neg)): 
            if ((var_list_y[index] < var_tol) and (numpy.abs(all_x[best_cu[0]] - all_x[best_cu[1]]) > 0.06)):
                mid_index = int((best_cu[1] - best_cu[0])/2) + best_cu[0]
                count = 1
                right_length = False
                while not right_length:
                    if (count > len(all_x)/2):
                        break
                    if (numpy.abs(all_x[mid_index + count] - all_x[mid_index - 1]) > 0.04):
                        right_length = True
                        break
                    count = count + 1
                best_cu = [mid_index - count, mid_index + count]

        elif ((app_direction == slopes_y_pos) or (app_direction == slopes_y_neg)): 
            if ((var_list_x[index] < var_tol) and (numpy.abs(all_y[best_cu[0]] - all_y[best_cu[1]]) > 0.06)):
                mid_index = int((best_cu[1] - best_cu[0])/2) + best_cu[0]
                count = 1
                right_length = False
                while not right_length:
                    if (count > len(all_x)/2):
                        break
                    if (numpy.abs(all_y[mid_index + count] - all_y[mid_index - 1]) > 0.04):
                        right_length = True
                        break
                    count = count + 1
                best_cu = [mid_index - count, mid_index + count]

        rospy.loginfo('Best cleaning peaks: ' + str(best_cu))
        

        """
        Publishing the cleaning unit trajectory and markers
        """

        timing = arm_trajectory.timing
        l_traj = arm_trajectory.lArm[:]


        max_points = 10
        interval = int(numpy.floor((best_cu[1] - best_cu[0])/max_points))


        timing_unit = []
        r_unit = []
        l_unit = []
        if (interval > 1):
            for i in range(best_cu[0],best_cu[1]):
                
                #timing_unit.append(rospy.Duration(i*0.2))
                if ((i == best_cu[0]) or (i == best_cu[1])):
                    timing_unit.append(timing[i])

                    r_unit.append(ArmState(ArmState.ROBOT_BASE,
                                    r_traj[i].ee_pose,
                                    r_traj[i].joint_pose, Object()))

                    l_unit.append(ArmState(ArmState.ROBOT_BASE,
                                    l_traj[i].ee_pose,
                                    l_traj[i].joint_pose, Object()))
                elif ( i%interval == 0):
                    timing_unit.append(timing[i])

                    r_unit.append(ArmState(ArmState.ROBOT_BASE,
                                    r_traj[i].ee_pose,
                                    r_traj[i].joint_pose, Object()))

                    l_unit.append(ArmState(ArmState.ROBOT_BASE,
                                    l_traj[i].ee_pose,
                                    l_traj[i].joint_pose, Object()))

        #TODO: find the offset between the corner of the table 
        #      and the starting point for the EE

        #rospy.loginfo('new table corner 4 ' + str(new_table[3].position.x) + str(new_table[3].position.y))

        #Subtract these offsets from cleaning units to position based on table corner
        #origin_offset_x = all_x[best_cu[0]] - new_table[1].position.x
        #origin_offset_y = all_y[best_cu[0]] - new_table[1].position.y


        """
        Find starting corner
        """

        corner = []
        old_corner = []
        if ((app_direction == slopes_x_pos) and pos_rep):
            rospy.loginfo("Corner 1")
            corner = [new_table[0].position.x, new_table[0].position.y]
            old_corner = [table_corner[0].position.x, table_corner[0].position.y]

        elif ((app_direction == slopes_x_neg) and pos_rep):
            rospy.loginfo("Corner 3")
            corner = [new_table[2].position.x, new_table[2].position.y]
            old_corner = [table_corner[2].position.x, table_corner[2].position.y]

        elif ((app_direction == slopes_x_pos) and not pos_rep):
            rospy.loginfo("Corner 2")
            corner = [new_table[1].position.x, new_table[1].position.y]
            old_corner = [table_corner[1].position.x, table_corner[1].position.y]

        elif ((app_direction == slopes_x_neg) and not pos_rep):
            rospy.loginfo("Corner 4")
            corner = [new_table[3].position.x, new_table[3].position.y]
            old_corner = [table_corner[3].position.x, table_corner[3].position.y]

        elif ((app_direction == slopes_y_pos) and pos_rep):
            rospy.loginfo("Corner 1")
            corner = [new_table[0].position.x, new_table[0].position.y]
            old_corner = [table_corner[0].position.x, table_corner[0].position.y]

        elif ((app_direction == slopes_y_neg) and pos_rep):
            rospy.loginfo("Corner 2")
            corner = [new_table[1].position.x, new_table[1].position.y]
            old_corner = [table_corner[1].position.x, table_corner[1].position.y]

        elif ((app_direction == slopes_y_pos) and not pos_rep):
            rospy.loginfo("Corner 3")
            corner = [new_table[2].position.x, new_table[2].position.y]
            old_corner = [table_corner[2].position.x, table_corner[2].position.y]

        elif ((app_direction == slopes_y_neg) and not pos_rep):
            rospy.loginfo("Corner 4")
            corner = [new_table[3].position.x, new_table[3].position.y]
            old_corner = [table_corner[3].position.x, table_corner[3].position.y]



        """
        Finding offsets
        """

        #Subtract this number from new table corner to find new start
        tool_offset_x = old_corner[0] - prev_start[0]
        tool_offset_y = old_corner[1] - prev_start[1]

        rospy.loginfo('Old corner: ' + str(old_corner))
        rospy.loginfo('New corner: ' + str(corner))
        rospy.loginfo('Offset between old/new corners: ' + str(old_corner[0] - corner[0]) + ', ' + str(old_corner[1] - corner[1]))

        rospy.loginfo('Previous start: ' + str(prev_start))


        new_start = [corner[0] - tool_offset_x, corner[1] - tool_offset_y]

        rospy.loginfo('New start: ' + str(new_start))
        rospy.loginfo('Offset between starts: ' + str(prev_start[0] - new_start[0]) + ', ' + str(prev_start[1] - new_start[1]))

        #Subtract this value from  
        # origin_offset_x = all_x[best_cu[0]] - new_start[0]
        # origin_offset_y = all_y[best_cu[0]] - new_start[1]


        origin_offset_x = all_x[best_cu[0]] - prev_start[0]
        origin_offset_y = all_y[best_cu[0]] - prev_start[1]

        #These offsets are stitching the individual cleaning units together
        app_offset_x = 0
        app_offset_y = 0
        rep_offset_x = 0 
        rep_offset_y = 0

        #Offset at beginning and end of rep for robot to lift arm
        z_offset = 0.10

        start_time = timing_unit[0]
        unit_length = len(timing_unit)
        for i in range(unit_length):
            timing_unit[i] = timing_unit[i] - start_time
        unit_duration = timing_unit[-1]

        if ((app_direction == slopes_x_pos) or (app_direction == slopes_x_neg)) :
            app_offset_x = all_x[best_cu[1]] - all_x[best_cu[0]]
            rospy.loginfo('Application dir offset: ' + str(app_offset_x))
            app_dist = numpy.abs(app_offset_x)
            rep_offset_y = rep_dist
   
        elif ((app_direction == slopes_y_pos) or (app_direction == slopes_y_neg)):
            app_offset_y = all_y[best_cu[1]] - all_y[best_cu[0]]
            rospy.loginfo('Application dir offset: ' + str(app_offset_y))
            app_dist = numpy.abs(app_offset_y)
            rep_offset_x = rep_dist



        
        # decide number of cleaning units
        number_units_app = int(numpy.floor(corner_dist_app/app_dist))
        number_units_rep = int(numpy.floor(corner_dist_rep/numpy.abs(rep_dist)))


        rospy.loginfo('Number of units_app: ' + str(number_units_app))
        rospy.loginfo('Number of units_rep: ' + str(number_units_rep))

        #TODO: decide number of cleaning units
        time_step = 0.02
        timing_gen = []
        r_traj_gen = []
        l_traj_gen = []

        num_time_offset = 1
        num_extra_time_offset = 1
        time_offset = 1
        last_timing_unit = None


        
        start_pose = Pose()
        
        start_pose.position.x = all_x[0]
        start_pose.position.y = all_y[0]
        start_pose.position.z = all_z[0]

        timing_gen.append(timing_unit[0])

        start_pose.orientation.x = r_traj[0].ee_pose.orientation.x
        start_pose.orientation.y = r_traj[0].ee_pose.orientation.y
        start_pose.orientation.z = r_traj[0].ee_pose.orientation.z
        start_pose.orientation.w = r_traj[0].ee_pose.orientation.w


        r_traj_gen.append(ArmState(ArmState.ROBOT_BASE,
                        start_pose,
                        r_traj[0].joint_pose, Object()))

        l_traj_gen.append(ArmState(ArmState.ROBOT_BASE,
                        l_traj[0].ee_pose,
                        l_traj[0].joint_pose, Object()))

        # if (number_units_rep > 1):
        #     number_units_rep = number_units_rep - 1

        # if (number_units_app > 1):
        #     number_units_app = number_units_app - 1




        for k in range(number_units_rep):
            

            for j in range(number_units_app):
                rospy.loginfo('Adding unit: ' + str(j))
                for i in range(unit_length):
                    #timing_unit.append(timing[len(timing) -1] + (timing[i] + timing[i - 1]))
                    # timing_unit.append(timing[len(timing) -1] + rospy.Duration(0.1))
                    
                    # timing_gen.append(timing_unit[i] + rospy.Duration((j*(unit_duration.to_sec()+time_step))+number_units_app*k*(unit_duration.to_sec()+time_step)))
                    # timing_gen.append(timing_unit[i] + rospy.Duration((j*(unit_duration.to_sec()+time_step))))


                    r_new_pose = Pose()
                    #rospy.loginfo('Original point: ' + str(r_unit[i].ee_pose.position.x) + ', ' + str(r_unit[i].ee_pose.position.y))
                    
                    #r_new_pose.position.x = r_unit[i].ee_pose.position.x + j*app_offset_x + k*rep_offset_x + tool_offset_x + new_start[0]
                    #r_new_pose.position.y = r_unit[i].ee_pose.position.y + j*app_offset_y + k*rep_offset_y + tool_offset_y + new_start[1]
                    # r_new_pose.position.x = r_unit[i].ee_pose.position.x + j*app_offset_x + k*rep_offset_x - origin_offset_x -  old_corner[0] + corner[0]
                    # r_new_pose.position.y = r_unit[i].ee_pose.position.y + j*app_offset_y + k*rep_offset_y - origin_offset_y -  old_corner[1] + corner[1]
                    r_new_pose.position.x = r_unit[i].ee_pose.position.x + j*app_offset_x + k*rep_offset_x - origin_offset_x -  old_corner[0] + corner[0]
                    r_new_pose.position.y = r_unit[i].ee_pose.position.y + j*app_offset_y + k*rep_offset_y - origin_offset_y -  old_corner[1] + corner[1]




                    if ((j == 0) and (i == 0)):
                        r_new_pose.position.z = r_unit[i].ee_pose.position.z + z_offset
                        #Very first point only
                        # if not timing_gen:
                        #     timing_gen.append(timing_unit[i] + rospy.Duration(time_offset*time_step + (j*(unit_duration.to_sec()+time_step))+number_units_app*k*(unit_duration.to_sec()+time_step)))  
                        #First point in rep
                        #else:
                        if (k == 0):
                            rospy.loginfo('Point is: ' + str(r_new_pose.position.x) + ', ' + str(r_new_pose.position.y))
                        num_extra_time_offset = num_time_offset + 50
                        num_time_offset = num_extra_time_offset
                        timing_gen.append(timing_unit[i] + rospy.Duration(time_offset*time_step*num_time_offset + (j*(unit_duration.to_sec()+time_step))+number_units_app*k*(unit_duration.to_sec()+time_step)))
                    #Last point in rep
                    elif ((j == (number_units_app - 1)) and (i == (unit_length -1)) ):
                        r_new_pose.position.z = r_unit[i].ee_pose.position.z + z_offset
                        num_extra_time_offset = num_time_offset + 20
                        timing_gen.append(timing_unit[i] + rospy.Duration(time_step*time_offset*num_extra_time_offset  + time_offset*time_step + (j*(unit_duration.to_sec()+time_step))+number_units_app*k*(unit_duration.to_sec()+time_step)))
                        
                    #All the middle points
                    else:
                        r_new_pose.position.z = r_unit[i].ee_pose.position.z
                        num_time_offset = num_time_offset + 0.3
                        timing_gen.append(timing_unit[i]  + rospy.Duration(time_offset*time_step*num_time_offset + (j*(unit_duration.to_sec()+time_step))+number_units_app*k*(unit_duration.to_sec()+time_step )))
                    r_new_pose.orientation.x = r_unit[i].ee_pose.orientation.x
                    r_new_pose.orientation.y = r_unit[i].ee_pose.orientation.y
                    r_new_pose.orientation.z = r_unit[i].ee_pose.orientation.z
                    r_new_pose.orientation.w = r_unit[i].ee_pose.orientation.w


                    #rospy.loginfo('New point: ' + str(r_new_pose.position.x) + ', ' + str(r_new_pose.position.y))

                    r_traj_gen.append(ArmState(ArmState.ROBOT_BASE,
                                    r_new_pose,
                                    r_unit[i].joint_pose, Object()))

                    l_traj_gen.append(ArmState(ArmState.ROBOT_BASE,
                                    l_unit[i].ee_pose,
                                    l_unit[i].joint_pose, Object()))

                    last_timing_unit = timing_unit[i] + rospy.Duration(time_step*time_offset*(num_extra_time_offset) + time_offset*time_step + (j*(unit_duration.to_sec()+time_step))+number_units_app*k*(unit_duration.to_sec()+time_step))
            
        
        end_pose = Pose()
        
        end_pose.position.x = all_x[0]
        end_pose.position.y = all_y[0]
        end_pose.position.z = all_z[0]



        timing_gen.append(last_timing_unit + rospy.Duration(2.0))

        end_pose.orientation.x = r_traj[0].ee_pose.orientation.x
        end_pose.orientation.y = r_traj[0].ee_pose.orientation.y
        end_pose.orientation.z = r_traj[0].ee_pose.orientation.z
        end_pose.orientation.w = r_traj[0].ee_pose.orientation.w    

        r_traj_gen.append(ArmState(ArmState.ROBOT_BASE,
                                    end_pose,
                                    r_traj[0].joint_pose, Object()))

        l_traj_gen.append(ArmState(ArmState.ROBOT_BASE,
                                    l_traj[0].ee_pose,
                                    l_traj[0].joint_pose, Object())) 




        traj_step = ActionStep()
        traj_step.type = ActionStep.ARM_TRAJECTORY
        
        n_points = len(timing_unit) #TODO
            
        traj_step.armTrajectory = ArmTrajectory(
            self.surface,
            r_traj_gen[:],
            l_traj_gen[:],
            timing_gen[:],
            arm_trajectory.rRefFrame,
            arm_trajectory.lRefFrame,
            arm_trajectory.rRefFrameObject,
            arm_trajectory.lRefFrameObject,
            [], 
            [],
            []
        )
            
        traj_step.gripperAction = GripperAction(GripperState.CLOSED,
                                        GripperState.CLOSED)

        # Create data structure and visualize
        self.session.create_execution_trajectory(traj_step,
                                        self.world.get_frame_list())

        # Execute
        execution_z_offset = 0.00
        #self.arms.start_execution(self.session.execution, execution_z_offset)

  

    def find_cleaning_unit(self, repetition_vals, application_vals, indices, positive_rep_dir, positive_app_dir):
        """
        Isolate individual cleaning unit from segment trajectory
        """

        if not indices:
            return []

        offset = indices[0]


        subset_rep_vals = []
        subset_app_vals = []
        for ind in indices:
            subset_app_vals.append(application_vals[ind])
            subset_rep_vals.append(repetition_vals[ind])
        #find peaks in subset_rep_vals  --- cleaning_peaks_indcies


        data_rep = subset_rep_vals
        data_app = subset_app_vals

        '''filtering'''
        # window = sgnl.general_gaussian(51, p=0.5, sig=20)
        # filtered = sgnl.fftconvolve(window, data)
        # filtered = (numpy.average(data) / numpy.average(filtered)) * filtered
        # filtered = numpy.roll(filtered, -25)
        ''' method 1 '''
        peakind = sgnl.find_peaks_cwt(data_rep, numpy.arange(1,20), noise_perc=10)
        ''' method 2 '''
        # peakind = sgnl.find_peaks_cwt(filtered, numpy.arange(100,200))
        ''' method 3: finding the maximum values from filtered curves: '''
        # peakind = sgnl.argrelmax(filtered, order=5)

        ''' this is for converting tuple peakind to an array: '''
        # peak_index = []
        # for item in peakind:
        #     peak_index.extend(item)

        # peakind = peakind_ini[1:len(peakind_ini)-1]

          # print peakind
        peak_data_rep = []
        peak_data_app = []
        for i in peakind:
            peak_data_rep.append(data_rep[i])
            peak_data_app.append(data_app[i])



        plt.subplot(2, 1, 1)
        plt.plot(data_rep)
        plt.plot(peakind, peak_data_rep, 'ro')
        # plt.xlabel('time stamps(-)')
        plt.ylabel('Repetiton direction (m)')

        plt.subplot(2, 1, 2)
        plt.plot(data_app)
        plt.plot(peakind, peak_data_app, 'go')
        plt.ylabel('Application direction (m)')

        plt.subplots_adjust(left=0.15)
        plt.xlabel('time stamps(-)')
        plt.show()

        cleaning_peaks_indices = peakind


        smallest_so_far = numpy.inf
        current_cu_peaks = []
        good_cleaning_peaks = []
        rejected_peak_pairs = []

        if (len(cleaning_peaks_indices) < 2):
            return []

        
        for i in range(len(cleaning_peaks_indices) - 1):
            diff = numpy.abs(subset_rep_vals[cleaning_peaks_indices[i]] - subset_rep_vals[cleaning_peaks_indices[i + 1]])
            rospy.loginfo('Peak values (rep): ' + str(subset_rep_vals[cleaning_peaks_indices[i]]) + ', ' + str(subset_rep_vals[cleaning_peaks_indices[i+1]]))
            # if (diff < smallest_so_far):
            #     smallest_so_far = diff
            #     current_cu_peaks = [cleaning_peaks_indices[i], cleaning_peaks_indices[i+ 1]]
            #     rospy.loginfo('Smaller diff found')
            current_cu_peaks.append([cleaning_peaks_indices[i], cleaning_peaks_indices[i+ 1]])

        for i in current_cu_peaks:
            app_peak_vals =  [subset_app_vals[i[0]],  subset_app_vals[i[1]]]
            if (positive_app_dir):
                rospy.loginfo('Positive app dir')
                if (app_peak_vals[0] < app_peak_vals[1]):
                    rospy.loginfo('First peak lower. Good.')
                    rospy.loginfo('Peaks: ' + str(app_peak_vals[0]) + ', ' + str(app_peak_vals[1]))
                    good_cleaning_peaks.append([x+offset for x in i])
                else:
                    rospy.loginfo('First peak higher. Bad.')
                    rospy.loginfo('Peaks: ' + str(app_peak_vals[0]) + ', ' + str(app_peak_vals[1]))
            else:
                rospy.loginfo('Negative app dir')
                if (app_peak_vals[0] > app_peak_vals[1]):
                    rospy.loginfo('First peak higher. Good.')
                    rospy.loginfo('Peaks: ' + str(app_peak_vals[0]) + ', ' + str(app_peak_vals[1]))
                    good_cleaning_peaks.append([x+offset for x in i])
                else:
                    rospy.loginfo('First peak lower. Bad.')
                    rospy.loginfo('Peaks: ' + str(app_peak_vals[0]) + ', ' + str(app_peak_vals[1]))
        return good_cleaning_peaks



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
        plt.subplot(3, 1, 1)
        plt.plot(range(n_points), all_x, 'r.-', markersize = 20)
        plt.ylabel('X (m)',fontsize=40 )
        plt.xlim(0, len(all_x))
        plt.tick_params(axis='x', labelsize=20)
        plt.tick_params(axis='y', labelsize=20) 

        plt.subplot(3, 1, 2)
        plt.plot(range(n_points), all_y, 'b.-', markersize = 20)
        plt.ylabel('Y (m)', fontsize=40)
        plt.xlim(0, len(all_y))
        plt.tick_params(axis='x', labelsize=20)
        plt.tick_params(axis='y', labelsize=20)

        plt.subplot(3, 1, 3)
        plt.plot(range(n_points), all_z, 'g.-', markersize = 20)
        plt.ylabel('Z (m)', fontsize=40)
        plt.xlim(0, len(all_z))
        plt.tick_params(axis='x', labelsize=20)
        plt.tick_params(axis='y', labelsize=20)

        # plt.subplot(4, 1, 4)
        # n, bins, patches = plt.hist(all_z, num_bins, normed=1, facecolor='yellow', alpha=0.5)
        # plt.xlabel('z (histogram bins)')
        # plt.ylabel('occurance')

        # Tweak spacing to prevent clipping of ylabel
        plt.subplots_adjust(left=0.15)
        plt.xlabel('Time stamps (-)',fontsize=50)
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

    def compute_repetition_direction(self, points_x, points_y):
        """ Returns repetition direction """
        n = len(points_x)
        t = list(xrange(n))
        slope_x = numpy.polyfit(t, points_x,1)[0]
        slope_y = numpy.polyfit(t, points_y,1)[0]

        if (slope_y < 0 and slope_x < 0):
            # return ERROR_NEGATIVE_SLOPES 
            if (numpy.abs(slope_y) > numpy.abs(slope_x)):
                rospy.loginfo('Y is the repetition direction.')
            elif (numpy.abs(slope_y) < numpy.abs(slope_x)):
                rospy.loginfo('X is the repetition direction.')    
        elif (slope_y > slope_x):
            rospy.loginfo('Y is the repetition direction.')
            # return Y_DIR
        elif (slope_x > slope_y):
            rospy.loginfo('X is the repetition direction.')
            # return X_DIR
        else:
            return ERROR_INDETERMINATE_DIR

    def find_outliers(self, data, m=2, height = False):
        d = numpy.abs(data - numpy.median(data))
        mdev = numpy.median(d)
        if (mdev):
            s = d/mdev 
        else:
            s = [0]*len(d)
        _i = 0
        outlier_indices = []
        outliers = []
        non = []
        non_indices = []
        for i in s:
            if (i > m):
                outlier_indices.append(_i)
                outliers.append(i)
            else:
                non.append(i)
                non_indices.append(_i)
            _i = _i + 1

        if ((len(outlier_indices) == len(data)/2) and (height == True)):
            if (numpy.mean(outliers) < numpy.mean(non)):
                outlier_indices = non_indices

        return outlier_indices


        # med = numpy.median(data)
        # med_ind = data.index(med)
        # less_than_med = []
        # for i in data:
        #     if (i <= med):
        #         less_than_med.append(i)
        # data.append(numpy.median(less_than_med))
        # d = numpy.abs(data - med)
        # mdev = numpy.median(d)
        # s = d/mdev if mdev else 0.
        # _i = 0
        # outlier_indices = []
        # for i in s:
        #     if (i > m):
        #         outlier_indices.append(_i)
        #     _i = _i + 1

        # contains_fake = outlier_indices.count(numpy.median(less_than_med))
        # if (contains_fake):
        #     outlier_indices.pop(len(outlier_indices) - 1)
        # return outlier_indices
