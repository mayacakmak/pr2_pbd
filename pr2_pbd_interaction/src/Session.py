'''Everything related to an experiment session'''

from ProgrammedAction import ProgrammedAction
import rospy
import os
import yaml
import threading
import rospkg
from pr2_pbd_interaction.msg import ExperimentState
from pr2_pbd_interaction.srv import GetExperimentState
from pr2_pbd_interaction.srv import GetExperimentStateResponse


class Session:
    '''This class holds and maintains experimental data'''

    def __init__(self, object_list, is_debug=False):

        self.current_experiment = None
        self._selected_step = 0
        self._object_list = object_list
        self.pose_set = dict()
        self.lock = threading.Lock()
        self._interaction_state = 'UNKNOWN'

        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState)
        rospack = rospkg.RosPack()
        self._path = rospack.get_path('pr2_pbd_interaction')
        self._pose_dir = self._path + '/data/'

        self._load_arm_poses()

        self.n_experiments = self._count_existing_experiments()
        rospy.loginfo(str(self.n_experiments) + ' experiments already exist.')

        if (self.n_experiments == 0):
            self.create_new_experiment()
        else:
            self.switch_to_experiment(self.n_experiments-1, object_list)


        rospy.Service('get_experiment_state', GetExperimentState,
                      self.get_experiment_state_cb)

    def create_new_experiment(self):

        self.lock.acquire()
        self.current_experiment = self.n_experiments
        self.n_experiments = self.n_experiments + 1
        self._data_dir = self._path + '/data/experiment' + str(self.current_experiment) + '/'
        self.actions = dict()
        self.current_action_index = 0

        #rospy.set_param('data_directory', self._data_dir)

        if (os.path.exists(self._data_dir)):
            rospy.logwarn('File already exists for new experiment, this should not happen.')
        else:
            os.mkdir(self._data_dir)

        self.lock.release()
        self._update_experiment_state()

    def switch_to_experiment(self, exp_number, object_list):

        self.lock.acquire()
        if (exp_number < self.n_experiments and exp_number >= 0):
            self.current_experiment = exp_number
            rospy.loginfo('Current experiment: ' + str(self.current_experiment))
            self._data_dir = self._path + '/data/experiment' + str(self.current_experiment) + '/'
            if (not os.path.exists(self._data_dir)):
                rospy.logwarn('Directory does not exists for existing experiment, this should not happen.')
            self._load_actions(object_list)
        else:
            rospy.logerr('Cannot switch to experiment ' + str(exp_number))

        self.lock.release()
        self._update_experiment_state()

    def _load_actions(self, object_list):
        self.current_action_index = 0
        self.actions = dict()

        pose_files = os.listdir(self._data_dir)
        for pose_file_name in pose_files:
            if pose_file_name.count('.') > 0:
                name = pose_file_name[0:pose_file_name.index('.')]
                extension = pose_file_name[(pose_file_name.index('.')+1):(len(pose_file_name)+1)]
                if (extension == 'bag'):
                    dummy_action = ProgrammedAction(name, self._selected_step_cb)
                    dummy_action.load(self._data_dir, name)
                    self.actions.update({self.current_action_index: dummy_action})
                    self.current_action_index = self.current_action_index + 1
                    print 'Loaded existing demo ', pose_file_name, 'with', dummy_action.n_frames(), 'frames.'

        # Stay at the last action..
        if (self.n_actions() > 0):
            self.current_action_index = self.n_actions() - 1
            self._current_action().initialize_viz(object_list)
        else:
            rospy.logwarn('Did not find any actions in experiment ' + str(self.current_experiment))

    def _count_existing_experiments(self):
        has_found_experiment = True
        n_experiments = 0
        while (has_found_experiment):
            data_dir = self._path + '/data/experiment' + str(n_experiments) + '/'
            if (not os.path.exists(data_dir)):
                has_found_experiment = False
            else:
                n_experiments = n_experiments + 1
        return n_experiments

    def _load_arm_poses(self):
        pose_files = os.listdir(self._pose_dir)
        for pose_file_name in pose_files:
            if pose_file_name.count('.') > 0:
                name = pose_file_name[0:pose_file_name.index('.')]
                extension = pose_file_name[(pose_file_name.index('.')+1):(len(pose_file_name)+1)]
                if (extension == 'bag'):
                    dummy_action = ProgrammedAction(0, self._selected_step_cb)
                    dummy_action.load(self._pose_dir, name)
                    self.pose_set[name] = dummy_action
                    print 'Loaded', pose_file_name, 'with', dummy_action.n_frames(), 'frames.'
        print 'Loaded ' + str(len(self.pose_set)) + ' poses.'

    def _selected_step_cb(self, selected_step):
        '''Updates the selected step when interactive
        markers are clicked on'''
        self._selected_step = selected_step
        self._update_experiment_state()

    def update_interaction_state(self, state):
        self._interaction_state = state
        self._update_experiment_state()

    def get_experiment_state_cb(self, dummy):
        ''' Response to the experiment state service call'''
        return GetExperimentStateResponse(self._get_experiment_state())

    def _update_experiment_state(self):
        ''' Publishes a message with the latest state'''
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        ''' Creates a message with the latest state'''
        self.lock.acquire()
        es = ExperimentState(
                    self.n_experiments,
                    self.current_experiment,
                    self.n_actions(),
                    self.current_action_index,
                    self._get_action_names(),
                    self.n_frames(),
                    self.frame_types(),
                    self._selected_step,
                    self._get_gripper_states(0),
                    self._get_gripper_states(1),
                    self._get_ref_frames(0),
                    self._get_ref_frames(1),
                    self._object_list,
                    self._interaction_state)
        self.lock.release()
        return es

    def _get_action_names(self):
        names = []
        for i in self.actions.keys():
            names.append(self.actions[i].name)
        return names

    def _get_ref_frames(self, arm_index):
        ''' Returns the reference frames for the steps of the
        current action in array format'''
        ref_frames = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            ref_frame = action.get_step_ref_frame(arm_index, i)
            ref_frames.append(ref_frame)
        return ref_frames

    def _get_gripper_states(self, arm_index):
        ''' Returns the gripper states for current action
        in array format'''
        gripper_states = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            gripper_state = action.get_step_gripper_state(arm_index, i)
            gripper_states.append(gripper_state)
        return gripper_states

    def select_action_step(self, step_id):
        ''' Makes the interactive marker for the indicated action
        step selected, by showing the 6D controls'''
        self.actions[self.current_action_index].select_step(step_id)
        self._selected_step = step_id

    def new_action(self, tool_id, object_list):
        '''Creates new action'''
        self.lock.acquire()
        tool_name = 'Tool' + str(tool_id)
        names = self._get_action_names()

        if (self.n_actions() > 0):
            self._current_action().reset_viz()

        if names.count(tool_name) == 0:
            self.current_action_index = self.n_actions()
            self.actions.update({self.current_action_index:
                                 ProgrammedAction(tool_name,
                                                  self._selected_step_cb)})
            no_demo_exists = True
        else:
            rospy.logwarn('Demonstration for this tool already exists.')
            self.current_action_index = self.actions.keys()[names.index(tool_name)]
            self._current_action().initialize_viz(object_list)
            no_demo_exists = False

        self.lock.release()
        self._update_experiment_state()
        return no_demo_exists

    def n_actions(self):
        '''Returns the number of actions programmed so far'''
        return len(self.actions)

    def current_action_has_demo(self):
        '''Returns the current action'''
        self.lock.acquire()
        ca = self._current_action()
        self.lock.release()
        return (ca.n_frames() > 0)

    def get_current_action(self):
        '''Returns the current action'''
        self.lock.acquire()
        ca = self._current_action()
        self.lock.release()
        return ca

    def _current_action(self):
        return self.actions[self.current_action_index]

    def clear_current_action(self):
        '''Removes all steps in the current action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].clear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def undo_clear(self):
        '''Undo the effect of clear'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].undoClear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def save_current_action(self):
        '''Save current action onto hard drive'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].save(self._data_dir)
        else:
            rospy.logwarn('No skills created yet.')

    def add_step_to_action(self, step, object_list):
        '''Add a new step to the current action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].add_action_step(step,
                                                                object_list)
        else:
            rospy.logwarn('No skills created yet.')
        self._object_list = object_list
        self._update_experiment_state()

    def delete_last_step(self):
        '''Removes the last step of the action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].delete_last_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def repeat_step(self):
        '''copies previous step'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].repeat_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()
        
    def resume_deleted_step(self):
        '''Resumes the deleted step'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].resume_deleted_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def is_current_tool(self, id):
        tool_name = 'Tool' + str(id)
        self.lock.acquire()
        current_tool_name = self._current_action().name
        self.lock.release()
        return (tool_name == current_tool_name)

    def switch_to_action(self, action_number, object_list):
        '''Switches to indicated action'''
        if (self.n_actions() > 0):
            if (action_number < self.n_actions() and action_number >= 0):
                self._current_action().reset_viz()
                self.current_action_index = action_number
                self._current_action().initialize_viz(object_list)
                success = True
            else:
                rospy.logwarn('Cannot switch to action '
                              + str(action_number))
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._object_list = object_list
        self._update_experiment_state()
        return success

    def next_action(self, object_list):
        '''Switches to next action'''
        if (self.n_actions() > 0):
            if (self.current_action_index < self.n_actions()):
                self._current_action().reset_viz()
                self.current_action_index += 1
                self._current_action().initialize_viz(object_list)
                success = True
            else:
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._object_list = object_list
        self._update_experiment_state()
        return success

    def previous_action(self, object_list):
        '''Switches to previous action'''
        if (self.n_actions() > 0):
            if (self.current_action_index > 1):
                self._current_action().reset_viz()
                self.current_action_index -= 1
                self._current_action().initialize_viz(object_list)
                success = True
            else:
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._object_list = object_list
        self._update_experiment_state()
        return success

    def n_frames(self):
        '''Returns the number of frames'''
        if (self.n_actions() > 0):
            return self.actions[self.current_action_index].n_frames()
        else:
            rospy.logwarn('No skills created yet.')
            return 0
    
    def frame_types(self):
        '''returns frame types'''
        if (self.n_actions() > 0):
            return self.actions[self.current_action_index].frame_types()
        else:
            rospy.logwarn('No skills created yet.')
            return []
