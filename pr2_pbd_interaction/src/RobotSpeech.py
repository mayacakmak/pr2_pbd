import roslib
#roslib.load_manifest('speakeasy');
roslib.load_manifest('sound_play');
import rospy

# ROS libraries
import actionlib
from actionlib_msgs.msg import *
#from speakeasy.msg import SpeakEasyTextToSpeech
from sound_play.msg import SoundRequest
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA

class TTSCommands:
    SAY  = 0;
    STOP = 1;
   
class Speech:
    TEST_RESPONSE = 'Microphone working.'
    SKILL_CREATED = 'Created action'
    RIGHT_ARM_RELEASED = 'Right arm relaxed'
    RIGHT_ARM_HOLDING = 'Right arm frozen'
    RIGHT_HAND_OPENING = 'Opening right hand'
    RIGHT_HAND_CLOSING = 'Closing right hand'
    LEFT_ARM_RELEASED = 'Left arm relaxed'
    LEFT_ARM_HOLDING = 'Left arm frozen'
    LEFT_HAND_OPENING = 'Opening left hand'
    LEFT_HAND_CLOSING = 'Closing left hand'
    STEP_RECORDED = 'Pose saved.'
    POSE_DELETED = 'Last pose deleted'
    POSE_RESUMED = 'Pose resumed'
    DELETED_SKILL = 'Deleted action'
    START_EXECUTION = 'Starting execution of action'
    EXECUTION_ENDED = 'Execution ended'
    SWITCH_SKILL = 'Switched to action'
    SKILL_EMPTY = 'Skill has no poses to delete.'
    EXECUTION_ERROR_NOIK = 'Cannot execute action'
    EXECUTION_ERROR_NOPOSES = 'Not enough poses in action'
    ERROR_NEXT_SKILL = 'No actions after action'
    ERROR_PREV_SKILL = 'No actions before action'
    ERROR_NO_SKILLS = 'No actions created yet.'
    ERROR_NOTHING_TO_UNDO = 'There is nothing to undo.'
    ERROR_NO_EXECUTION = 'No executions in progress.'
    EXECUTION_PREEMPTED = 'Stopping execution.'
    RIGHT_HAND_ALREADY_OPEN = 'Right hand is already open.'
    LEFT_HAND_ALREADY_OPEN = 'Left hand is already open.'
    RIGHT_HAND_ALREADY_CLOSED = 'Right hand is already closed.'
    LEFT_HAND_ALREADY_CLOSED = 'Left hand is already closed.'
    RIGHT_ARM_ALREADY_HOLDING = 'Right arm is already frozen.'
    LEFT_ARM_ALREADY_HOLDING = 'Left arm is already frozen.'
    RIGHT_ARM_ALREADY_RELEASED = 'Right arm is already relaxed.'
    LEFT_ARM_ALREADY_RELEASED = 'Left arm is already relaxed.'
    SKILL_CLEARED = 'All poses deleted.'
    LAST_POSE_DELETED = 'Last pose deleted.'
    ALL_POSES_RESUMED = 'All poses resumed.'
    START_STATE_RECORDED = 'Start state recorded.'
    OBJECT_NOT_DETECTED = 'No objects were detected.'
    ACTION_SAVED = 'Saved Action '
    ALREADY_EDITING = 'Already in editing mode.'
    SWITCH_TO_EDIT_MODE = 'Switched to edit mode.'
    ERROR_NOT_IN_EDIT = ' has been saved. Say, edit action, to make changes.'
    ACTION_ALREADY_STARTED = 'Action already started. Say, delete all steps, to start over.'
    ALREADY_RECORDING_MOTION = 'Already recording motion.'
    STARTED_RECORDING_MOTION = 'Started recording motion.'
    STOPPED_RECORDING_MOTION = 'Stopped recording motion.'
    MOTION_NOT_RECORDING = 'Not currently recording motion.'
    STOPPING_EXECUTION = 'Execution stopped.'

class RobotSpeech:
    "Things that the robot say"
    def __init__(self):
        #self.speechOutputSpeakeasy = rospy.Publisher('speakeasy_text_to_speech_req', SpeakEasyTextToSpeech)
        self.speechOutputSoundplay = rospy.Publisher('robotsound', SoundRequest)
        self.markerPublisher = rospy.Publisher('visualization_marker', Marker)
        
    def say(self, text, useSpeakeasy=False):
	useSpeakeasy = False        
	if (useSpeakeasy):
#            ttsRequestMsg = SpeakEasyTextToSpeech()
#            ttsRequestMsg.command = TTSCommands.SAY
#            ttsRequestMsg.text    = text
#            ttsRequestMsg.engineName = 'cepstral'
#            ttsRequestMsg.voiceName = 'David'
#            self.speechOutputSpeakeasy.publish(ttsRequestMsg)
            self.sayInRViz(text)
        else:
#            self.speechOutputSoundplay.publish(SoundRequest(command=SoundRequest.SAY, arg=text))
            self.sayInRViz(text)
            
    def sayInRViz(self, text):
        m = Marker(type=Marker.TEXT_VIEW_FACING, id=1000, lifetime=rospy.Duration(1.5), 
                    pose=Pose(Point(0.5,0.5,1.45), Quaternion(0,0,0,1)),
                    scale=Vector3(0.06,0.06,0.06), header=Header(frame_id='base_link'),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
        self.markerPublisher.publish(m)

    def stopSaying(self):
        ttsRequestMsg = SpeakEasyTextToSpeech()
        ttsRequestMsg.command = TTSCommands.STOP
        self.speechOutputSpeakeasy.publish(ttsRequestMsg)


