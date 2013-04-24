import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('pr2_social_gaze')
roslib.load_manifest('sound_play')

from pr2_social_gaze.msg import *
from RobotSpeech import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import os

roslib.load_manifest('pr2_pbd_interaction')
from pr2_pbd_interaction.msg import RobotSound

class Response:
    "Unit of interaction, explains how to respond to a speech command"
    # Static stuff
    gazeActionClient = None
    robotSpeech = None
    soundPlayClient = None
    soundsDir = os.popen('rospack find pr2_pbd_interaction').read()
    soundsDir = soundsDir[0:len(soundsDir)-1] + "/sounds/"
    
    glanceActions = [GazeGoal.GLANCE_RIGHT_EE, GazeGoal.GLANCE_LEFT_EE]
    followActions = [GazeGoal.FOLLOW_RIGHT_EE, GazeGoal.FOLLOW_LEFT_EE]
    openResponses = [Speech.RIGHT_HAND_OPENING, Speech.LEFT_HAND_OPENING]
    closeResponses = [Speech.RIGHT_HAND_CLOSING, Speech.LEFT_HAND_CLOSING]
    releaseResponses = [Speech.RIGHT_ARM_RELEASED, Speech.LEFT_ARM_RELEASED]
    holdResponses = [Speech.RIGHT_ARM_HOLDING, Speech.LEFT_ARM_HOLDING]
    alreadyOpenResponses = [Speech.RIGHT_HAND_ALREADY_OPEN, Speech.LEFT_HAND_ALREADY_OPEN]
    alreadyClosedResponses = [Speech.RIGHT_HAND_ALREADY_CLOSED, Speech.LEFT_HAND_ALREADY_CLOSED]
    alreadyHoldingResponses = [Speech.RIGHT_ARM_ALREADY_HOLDING, Speech.LEFT_ARM_ALREADY_HOLDING]
    alreadyReleasedResponses = [Speech.RIGHT_ARM_ALREADY_RELEASED, Speech.LEFT_ARM_ALREADY_RELEASED]
    
    allSounds = [RobotSound.ALL_POSES_DELETED, RobotSound.ERROR, RobotSound.MICROPHONE_WORKING, RobotSound.POSE_SAVED, 
                 RobotSound.START_TRAJECTORY, RobotSound.CREATED_ACTION, RobotSound.EXECUTION_ENDED, RobotSound.OTHER, 
                 RobotSound.STARTING_EXECUTION, RobotSound.SUCCESS]
    
    def __init__(self, functionToCall, functionParam):
        self.functionToCall = functionToCall
        self.functionParam = functionParam

        if (Response.gazeActionClient == None):
            Response.gazeActionClient = actionlib.SimpleActionClient('gaze_action', GazeAction)
            Response.gazeActionClient.wait_for_server()
            
        if (Response.robotSpeech == None):
            Response.robotSpeech = RobotSpeech()
        
        if (Response.soundPlayClient == None):
            Response.soundPlayClient = SoundClient()

    def respond(self):
        speechResponse,gazeResponse  = self.functionToCall(self.functionParam)
        # Speech response
        if (speechResponse != None):
            Response.say(speechResponse)
            self.respondWithSound(speechResponse)
        # Gaze response
        if (gazeResponse != None):
            Response.performGazeAction(gazeResponse)

    @staticmethod
    def performGazeAction(gazeAction):
        goal = GazeGoal()
        goal.action = gazeAction
        Response.gazeActionClient.send_goal(goal)

    @staticmethod
    def lookAtPoint(point):
        Response.gazeActionClient.send_goal(GazeGoal(GazeGoal.LOOK_AT_POINT, point))

    @staticmethod
    def say(speechResponse):
        Response.robotSpeech.say(speechResponse)
        
    def respondWithSound(self, speechResponse):
        if (speechResponse == Speech.STEP_RECORDED):
            Response.playSound(RobotSound.POSE_SAVED)

        elif (speechResponse == Speech.TEST_RESPONSE):
            Response.playSound(RobotSound.MICROPHONE_WORKING) 
        
        elif (speechResponse == Speech.SKILL_CLEARED):
            Response.playSound(RobotSound.ALL_POSES_DELETED)
        
        elif (Speech.START_EXECUTION in speechResponse):
            Response.playSound(RobotSound.STARTING_EXECUTION)
        
        elif (speechResponse == Speech.EXECUTION_ENDED):
            Response.playSound(RobotSound.EXECUTION_ENDED)

        elif (speechResponse == Speech.STARTED_RECORDING_MOTION):
            Response.playSound(RobotSound.START_TRAJECTORY)

        elif (Speech.SKILL_CREATED in speechResponse):
            Response.playSound(RobotSound.CREATED_ACTION)
            
        elif (speechResponse == Speech.START_STATE_RECORDED or 
              speechResponse == Speech.STOPPED_RECORDING_MOTION or 
              Speech.SWITCH_SKILL in speechResponse):
            Response.playSound(RobotSound.SUCCESS)
        
        elif (speechResponse == Speech.OBJECT_NOT_DETECTED or 
              speechResponse == Speech.MOTION_NOT_RECORDING or
              speechResponse == Speech.ERROR_NEXT_SKILL or
              speechResponse == Speech.ERROR_NO_EXECUTION or
              speechResponse == Speech.ERROR_NO_SKILLS or
              speechResponse == Speech.ERROR_NOT_IN_EDIT or
              speechResponse == Speech.ERROR_PREV_SKILL or
              speechResponse == Speech.EXECUTION_ERROR_NOIK or
              speechResponse == Speech.EXECUTION_ERROR_NOPOSES or
              speechResponse == Speech.EXECUTION_PREEMPTED or
              speechResponse == Speech.RIGHT_HAND_ALREADY_OPEN or
              speechResponse == Speech.LEFT_HAND_ALREADY_OPEN or
              speechResponse == Speech.RIGHT_HAND_ALREADY_CLOSED or
              speechResponse == Speech.LEFT_HAND_ALREADY_CLOSED or
              speechResponse == Speech.RIGHT_ARM_ALREADY_HOLDING or
              speechResponse == Speech.RIGHT_ARM_ALREADY_RELEASED or
              speechResponse == Speech.LEFT_ARM_ALREADY_HOLDING or
              speechResponse == Speech.LEFT_ARM_ALREADY_RELEASED):
            Response.playSound(RobotSound.ERROR)
        
        else:
            Response.playSound(RobotSound.OTHER) 
            
    @staticmethod
    def playSound(requestedSound):
        if (requestedSound in Response.allSounds):
            Response.soundPlayClient.playWave(Response.soundsDir + requestedSound + '.wav')
        else:
            Response.soundPlayClient.playWave(Response.soundsDir + 'OTHER.wav')        
        
        
        
        
        