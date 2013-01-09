import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('pr2_social_gaze')

from pr2_social_gaze.msg import *
from RobotSpeech import *

class Response:
    "Unit of interaction, explains how to respond to a speech command"
    # Static stuff
    gazeActionClient = None
    robotSpeech = None

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
    
    def __init__(self, functionToCall, functionParam):
        self.functionToCall = functionToCall
        self.functionParam = functionParam

        if (Response.gazeActionClient == None):
            Response.gazeActionClient = actionlib.SimpleActionClient('gaze_action', GazeAction)
            Response.gazeActionClient.wait_for_server()    
        if (Response.robotSpeech == None):
            Response.robotSpeech = RobotSpeech()

    def respond(self):
        speechResponse,gazeResponse  = self.functionToCall(self.functionParam)
        # Speech response
        if (speechResponse != None):
            Response.say(speechResponse)
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