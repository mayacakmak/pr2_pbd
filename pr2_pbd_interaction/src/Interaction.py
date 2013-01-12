import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('speech_recognition')
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import sys,os

# ROS libraries
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String

# Local stuff
from ProgrammedAction import *
from World import *
from RobotSpeech import Speech
from Session import *
from Response import *
from ActionStepMarker import *
from Arms import *
from pr2_pbd_interaction.msg import *
from speech_recognition.msg import Command
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA

COLOR_RED = "\033[31m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_NORMAL = "\033[0m"

class Interaction:
    "Finite state machine for the human interaction"
    def __init__(self):

        self.arms = Arms()
        self.session = Session(isDebug=True)
        self.world = World()

        self.stateOutput = rospy.Publisher('interaction_state', String)
        self.visualizationOutput = rospy.Publisher('visualization_marker_array', MarkerArray)
        rospy.Subscriber('recognized_command', Command, self.handle_speech_command)

        self.prevArmPoses = None
        self.responses = {Command.TEST_MICROPHONE: Response(self.emptyResponse, [Speech.TEST_RESPONSE, GazeGoal.NOD]),
                          Command.RELAX_RIGHT_ARM: Response(self.relaxArm, 0),
                          Command.RELAX_LEFT_ARM: Response(self.relaxArm, 1),
                          Command.OPEN_RIGHT_HAND: Response(self.openHand, 0),
                          Command.OPEN_LEFT_HAND: Response(self.openHand, 1),
                          Command.CLOSE_RIGHT_HAND: Response(self.closeHand, 0),
                          Command.CLOSE_LEFT_HAND: Response(self.closeHand, 1),
                          Command.STOP_EXECUTION: Response(self.stopExecution, None),
                          Command.UNDO: Response(self.undo, None),
                          Command.DELETE_ALL_STEPS: Response(self.deleteAllSteps, None),
                          Command.DELETE_LAST_STEP: Response(self.deleteLastStep, None),
                          Command.FREEZE_RIGHT_ARM: Response(self.freezeArm, 0),
                          Command.FREEZE_LEFT_ARM: Response(self.freezeArm, 1),
                          Command.RECORD_OBJECT_POSE: Response(self.recordObjectPose, None),
                          Command.CREATE_NEW_ACTION: Response(self.createAction, None),
                          Command.EXECUTE_ACTION: Response(self.executeProgrammedAction, None),
                          Command.NEXT_ACTION: Response(self.nextProgrammedAction, None),
                          Command.PREV_ACTION: Response(self.prevProgrammedAction, None),
                          Command.SAVE_ACTION: Response(self.saveProgrammedAction, None),
                          Command.EDIT_ACTION: Response(self.editProgrammedAction, None),
                          Command.SAVE_POSE: Response(self.saveArmStep, None),

                          Command.START_RECORDING_MOTION: Response(self.startRecordingMotion, None),
                          Command.STOP_RECORDING_MOTION: Response(self.stopRecordingMotion, None)
                          }
        
        self.isProgramming = False
        self.isRecordingMotion = False
        self.armTrajectory = None
        self.trajectoryStartTime = None
        self.undoFunction = None
        
        rospy.loginfo('Interaction initialized.')
        
## Functions for responding to commands

    def openHand(self, armIndex):
        if self.arms.setGripperState(armIndex, GripperState.OPEN):
            speechResponse = Response.openResponses[armIndex]
            if (self.isProgramming and self.session.nProgrammedActions() > 0):
                self.saveGripperStep(armIndex, GripperState.OPEN)
                speechResponse = speechResponse + ' ' + Speech.STEP_RECORDED
            return [speechResponse, Response.glanceActions[armIndex]]
        else:
            return [Response.alreadyOpenResponses[armIndex], Response.glanceActions[armIndex]]
    
    def closeHand(self, armIndex, deletePose=False):
        if self.arms.setGripperState(armIndex, GripperState.CLOSED):
            speechResponse = Response.closeResponses[armIndex]
            if (self.isProgramming and self.session.nProgrammedActions() > 0):
                self.saveGripperStep(armIndex, GripperState.CLOSED)
                speechResponse = speechResponse + ' ' + Speech.STEP_RECORDED
            return [speechResponse, Response.glanceActions[armIndex]]
        else:
            return [Response.alreadyClosedResponses[armIndex], Response.glanceActions[armIndex]]

    def relaxArm(self, armIndex):
        if self.arms.setArmMode(armIndex, ArmMode.RELEASE):
            return [Response.releaseResponses[armIndex], Response.glanceActions[armIndex]]
        else:
            return [Response.alreadyReleasedResponses[armIndex], Response.glanceActions[armIndex]]

    def freezeArm(self, armIndex):
        if self.arms.setArmMode(armIndex, ArmMode.HOLD):
            return [Response.holdResponses[armIndex], Response.glanceActions[armIndex]]
        else:
            return [Response.alreadyHoldingResponses[armIndex], Response.glanceActions[armIndex]]

    def editProgrammedAction(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.isProgramming):
                return [Speech.ALREADY_EDITING, GazeGoal.LOOK_FORWARD]
            else:
                self.isProgramming = True
                return [Speech.SWITCH_TO_EDIT_MODE, GazeGoal.NOD]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]
    
    def saveProgrammedAction(self, param=None):
        self.isProgramming = False
        return [Speech.ACTION_SAVED + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.NOD]
    
    def createAction(self, param=None):
        self.session.newProgrammedAction()
        self.isProgramming = True        
        return [Speech.SKILL_CREATED + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.NOD]
   
    def nextProgrammedAction(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if self.session.nextProgrammedAction():
                return [Speech.SWITCH_SKILL + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.NOD]
            else:
                return [Speech.ERROR_NEXT_SKILL + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.SHAKE]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]
        
    def prevProgrammedAction(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if self.session.previousProgrammedAction():
                return [Speech.SWITCH_SKILL + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.NOD]
            else:
                return [Speech.ERROR_PREV_SKILL + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.SHAKE]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def deleteLastStep(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.isProgramming):
                if self.session.nFrames() > 0:
                    self.session.deleteLastStep()
                    self.undoFunction = self.resumeLastPose
                    return [Speech.LAST_POSE_DELETED, GazeGoal.NOD]
                else:
                    return [Speech.SKILL_EMPTY, None]
            else:
                return ['Action ' + str(self.session.currentProgrammedActionIndex) + Speech.ERROR_NOT_IN_EDIT, GazeGoal.LOOK_FORWARD]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def deleteAllSteps(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.isProgramming):
                if self.session.nFrames() > 0:
                    self.session.clearProgrammedAction()
                    self.undoFunction = self.resumeAllPoses
                    return [Speech.SKILL_CLEARED, GazeGoal.NOD]
                else:
                    return [Speech.SKILL_EMPTY, None]
            else:
                return ['Action ' + str(self.session.currentProgrammedActionIndex) + Speech.ERROR_NOT_IN_EDIT, GazeGoal.LOOK_FORWARD]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def undo(self, param=None):
        if (self.undoFunction == None):
            return [Speech.ERROR_NOTHING_TO_UNDO, GazeGoal.SHAKE]
        else:
            return self.undoFunction()
        
    def resumeAllPoses(self):
        self.session.undoClearProgrammedAction()
        return [Speech.ALL_POSES_RESUMED, GazeGoal.NOD]
        
    def resumeLastPose(self):
        self.session.resumeDeletedPose()
        return [Speech.POSE_RESUMED, GazeGoal.NOD]
        
    def stopExecution(self, param=None):
        if (self.arms.isExecuting()):
            self.arms.stopExecution()
            return [None, None]
        else:
            return [Speech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]
        
    def startAndSave(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.session.nFrames() == 0):
                self.isProgramming = True
                self.saveArmStep()
                return [Speech.FIRST_STEP_RECORDED, None]
            else:
                return [Speech.ACTION_ALREADY_STARTED, GazeGoal.LOOK_FORWARD]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]
        
    def saveGripperStep(self, armIndex, gripperState):
        step = ActionStep()
        step.type = ActionStep.GRIPPER_TARGET
        states = [GripperState(self.arms.getGripperState(0)), GripperState(self.arms.getGripperState(1))]
        states[armIndex].state = gripperState
        step.gripperTarget = GripperTarget(states[0], states[1])
        self.session.addStepToProgrammedAction(step)
        
    def startRecordingMotion(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.isProgramming):
                if (not self.isRecordingMotion):
                    #self.saveArmStep() # Add a waypoint at the starting point of the trajectory
                    self.isRecordingMotion = True
                    self.armTrajectory = ArmTrajectory()
                    self.trajectoryStartTime = rospy.Time.now()
                    return [Speech.STARTED_RECORDING_MOTION, GazeGoal.LOOK_FORWARD]
                else:
                    return [Speech.ALREADY_RECORDING_MOTION, GazeGoal.SHAKE]
            else:
                return ['Action ' + str(self.session.currentProgrammedActionIndex) + Speech.ERROR_NOT_IN_EDIT, GazeGoal.LOOK_FORWARD]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]
        
    def stopRecordingMotion(self, param=None):
        if (self.isRecordingMotion):
            self.isRecordingMotion = False
            armTrajectoryStep = ActionStep()
            armTrajectoryStep.type = ActionStep.ARM_TRAJECTORY
            
            waitedTime = self.armTrajectory.timing[0]
            for i in range(len(self.armTrajectory.timing)):
                self.armTrajectory.timing[i] -= waitedTime
                self.armTrajectory.timing[i] += rospy.Duration(0.1)

            self.fixTrajectoryReferenceFrames()
            armTrajectoryStep.armTrajectory = ArmTrajectory(self.armTrajectory.rArm[:], self.armTrajectory.lArm[:], 
                                                            self.armTrajectory.timing[:], self.armTrajectory.rRefFrame, self.armTrajectory.lRefFrame)
            self.session.addStepToProgrammedAction(armTrajectoryStep)
            self.armTrajectory = None
            self.trajectoryStartTime = None
            return [Speech.STOPPED_RECORDING_MOTION + ' ' + Speech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [Speech.MOTION_NOT_RECORDING, GazeGoal.SHAKE]
        
    def fixTrajectoryReferenceFrames(self):
        rRefFrame = self.findDominantRefFrame(self.armTrajectory.rArm)
        lRefFrame = self.findDominantRefFrame(self.armTrajectory.lArm)
        
        startFrame = self.session.getLastStepOfProgrammedAction()
        self.armTrajectory.rArm[0] = World.convertRefFrame(rRefFrame, self.armTrajectory.rArm[0], startFrame.armTarget.rArm)
        self.armTrajectory.lArm[0] = World.convertRefFrame(lRefFrame, self.armTrajectory.lArm[0], startFrame.armTarget.lArm)
        for i in range(1, len(self.armTrajectory.timing)):
            self.armTrajectory.rArm[i] = World.convertRefFrame(rRefFrame, self.armTrajectory.rArm[i], self.armTrajectory.rArm[i-1])
            self.armTrajectory.lArm[i] = World.convertRefFrame(lRefFrame, self.armTrajectory.lArm[i], self.armTrajectory.lArm[i-1])
        self.armTrajectory.rRefFrame = rRefFrame
        self.armTrajectory.lRefFrame = lRefFrame
        
    def findDominantRefFrame(self, armTraj):
        refFrameCounts = [0]*5
        for i in range(len(armTraj)):
            refFrameCounts[armTraj[i].refFrame] += 1
        refFrameN = max(refFrameCounts[0:4]) # Excluding 'NOT_MOVING'
        if (refFrameN > 0):
            return refFrameCounts[0:4].index(refFrameN)
        else:
            return ArmState.NOT_MOVING # Because the arm was never moving

    def saveStateToArmTrajectory(self):
        if (self.armTrajectory != None):
            states = self.getArmStates()
            if (states[0].refFrame != ArmState.NOT_MOVING or states[1].refFrame != ArmState.NOT_MOVING):
                self.armTrajectory.rArm.append(states[0])
                self.armTrajectory.lArm.append(states[1])
                self.armTrajectory.timing.append(rospy.Time.now() - self.trajectoryStartTime)

    def saveArmStep(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.isProgramming):
                states = self.getArmStates()
                step = ActionStep()
                step.type = ActionStep.ARM_TARGET
                step.armTarget = ArmTarget(states[0], states[1], 0.2, 0.2)
                self.session.addStepToProgrammedAction(step)
                                
                return [Speech.STEP_RECORDED, GazeGoal.LOOK_FORWARD]
            else:
                return ['Action ' + str(self.session.currentProgrammedActionIndex) + Speech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def getArmStates(self):
        absEEPoses = [self.arms.getEndEffectorState(0), self.arms.getEndEffectorState(1)]
        jointPoses = [self.arms.getDemoJointState(0), self.arms.getDemoJointState(1)]
        
        relEEPoses = [None, None]
        thresholdMoved = 0.002
        thresholdFar = 0.75
        states = [None, None]

        for armIndex in [0,1]:
            if ((self.session.nFrames() > 0) and (self.prevArmPoses != None) 
                and (Arm.eeDistance(self.prevArmPoses[armIndex], absEEPoses[armIndex]) < thresholdMoved)):
                # No actions
                states[armIndex] = ArmState(ArmState.NOT_MOVING, absEEPoses[armIndex], jointPoses[armIndex])
                # rospy.loginfo('Arm did not move: ' + str(armIndex))
            else:
                if ((self.world.poses == None) or 
                    (Arm.eeDistance(self.world.poses[self.world.index], absEEPoses[armIndex]) > thresholdFar)):
                    # Absolute
                    states[armIndex] = ArmState(ArmState.ROBOT_BASE, absEEPoses[armIndex], jointPoses[armIndex])
                else:
                    # Relative
                    relEEPoses[armIndex] = World.transform(absEEPoses[armIndex], '/base_link', '/task_object')
                    states[armIndex] = ArmState(ArmState.OBJECT, relEEPoses[armIndex], jointPoses[armIndex])
            
        self.prevArmPoses = absEEPoses
        return states
        
    def executeProgrammedAction(self, param=None):
        if (self.session.nProgrammedActions() > 0):
            if (self.session.nFrames() > 1):
                self.session.saveProgrammedAction()
                pAction = self.session.getProgrammedAction()
                
                if (pAction.requiresObject()):
                    if (self.world.updateTaskObjectPose()):
                        self.session.getProgrammedAction().updateInteractiveMarkers()
                        self.arms.startExecution(pAction)
                    else:
                        return [Speech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]
                else:
                    self.arms.startExecution(pAction)

                return [Speech.START_EXECUTION + ' ' + str(self.session.currentProgrammedActionIndex), None]
            else:
                return [Speech.EXECUTION_ERROR_NOPOSES + ' ' + str(self.session.currentProgrammedActionIndex), GazeGoal.SHAKE]
        else:
            return [Speech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

## Speech command callback

    def handle_speech_command(self, command):
        if command.command in self.responses.keys():
            rospy.loginfo(COLOR_GREEN + 'Calling response for command '+ command.command + COLOR_NORMAL)
            response = self.responses[command.command]
            if (not self.arms.isExecuting()):
                if (self.undoFunction != None):
                    response.respond()
                    self.undoFunction = None
                else:
                    response.respond()
                self.stateOutput.publish(self.session.getCurrentStatus())
            else:
                if command.command == Command.STOP_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring command sent during execution: '+ command.command)
        else:
            rospy.logwarn(COLOR_GREEN + 'This command (' + command.command + ') is unknown.' + COLOR_NORMAL);


## Update loop

    def update(self):
        self.arms.update()
        
        if (self.arms.executionStatus != ExecutionStatus.NOT_EXECUTING):
            if (self.arms.executionStatus != ExecutionStatus.EXECUTING):
                self.completeExecution()
        
        if (self.isRecordingMotion):
            self.saveStateToArmTrajectory()

        if (self.session.nProgrammedActions() > 0):
            self.session.getProgrammedAction().updateVisualization()

        self.world.update()
        time.sleep(0.1)

## Utility/helper functions

    def completeExecution(self):
        if (self.arms.executionStatus == ExecutionStatus.SUCCEEDED):
            speechResponse = Speech.EXECUTION_ENDED
        elif (self.arms.executionStatus == ExecutionStatus.PREEMPTED):
            speechResponse = Speech.EXECUTION_PREEMPTED
        else:
            speechResponse = Speech.EXECUTION_ERROR_NOIK
        
        self.arms.executionStatus = ExecutionStatus.NOT_EXECUTING
        Response.performGazeAction(GazeGoal.LOOK_FORWARD)
        Response.say(speechResponse)
             
    def recordObjectPose(self, param=None):
        if (self.world.updateTaskObjectPose()):
            if (self.session.nProgrammedActions() > 0):
                self.session.getProgrammedAction().updateInteractiveMarkers()
            return [Speech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            return [Speech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]

    def saveExperimentState(self):
        self.session.saveExperimentState()
        
    def emptyResponse(self, responses):
        return responses;
    
