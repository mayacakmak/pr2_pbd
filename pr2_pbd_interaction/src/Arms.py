# Dependencies
import roslib
roslib.load_manifest('rospy')
import rospy

roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('pr2_social_gaze')

# Generic libraries
import time, sys
from numpy import *

# ROS libraries
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Local
from Arm import *
from Response import *
from pr2_social_gaze.msg import *
from World import *

class ExecutionStatus:
    PREEMPTED = 0
    SUCCEEDED = 1
    NO_IK = 2
    OBSTRUCTED = 3
    NOT_EXECUTING = 4
    EXECUTING = 5
    CONDITION_ERROR = 6

class Arms:
    
    def __init__(self):
        
        armRight = Arm(ArmSide.RIGHT)
        armLeft = Arm(ArmSide.LEFT)
        self.arms = [armRight, armLeft]
        self.attendedArm = -1
        self.pAction = None
        self.preempt = False
        
        rospy.loginfo('Arms have been initialized.')

        self.arms[0].setMode(ArmMode.HOLD)
        self.arms[1].setMode(ArmMode.HOLD)
        self.arms[0].updateGripperState()
        self.arms[1].updateGripperState()
        self.closeGripper(ArmSide.RIGHT)
        self.closeGripper(ArmSide.LEFT)
        self.executionStatus = ExecutionStatus.NOT_EXECUTING

    def setArmMode(self, armIndex, mode):
        if (mode == self.getMode(armIndex)):
            # Already in that mode
            return False
        else:
            self.arms[armIndex].setMode(mode)
            return True

    def setGripperState(self, armIndex, gripperState):
        if (gripperState == self.getGripperState(armIndex)):
            # Already in that mode
            return False
        else:
            if (gripperState == GripperState.OPEN):
                self.openGripper(armIndex)
            else:
                self.closeGripper(armIndex)
        return True

    def getDemoJointState(self, armIndex):
        return self.arms[armIndex].getJointPositions()            
    
    def openGripper(self, armIndex, wait=False):
        self.arms[armIndex].openGripper(wait=wait)
        
    def closeGripper(self, armIndex, wait=False):
        self.arms[armIndex].closeGripper(wait=wait)
        
    def getGripperState(self, armIndex):
        return self.arms[armIndex].getGripperState()

    def getEndEffectorState(self, armIndex):
        return self.arms[armIndex].getEndEffectorState()
    
    def getMode(self, armIndex):
        return self.arms[armIndex].armMode
    
    def isExecuting(self):
        return (self.executionStatus == ExecutionStatus.EXECUTING)

    def startExecution(self, pAction):
        # This will take long, create a thread
        self.pAction = pAction.copy()
        self.preempt = False
        thread = threading.Thread(group=None, target=self.executeProgrammedAction, name='skill_execution_thread')
        thread.start()

    def stopExecution(self):
        self.preempt = True
    
    def solveIK4ProgrammedAction(self):
        # Go over steps of the action
        for i in range(self.pAction.nFrames()):
            # For each step check step type
            # If arm target action
            if (self.pAction.seq.seq[i].type == ActionStep.ARM_TARGET):
                # Find frames that are relative and convert to absolute
                self.pAction.seq.seq[i].armTarget.rArm, foundSolutionR = self.solveIK4ArmState(self.pAction.seq.seq[i].armTarget.rArm)
                self.pAction.seq.seq[i].armTarget.lArm, foundSolutionL = self.solveIK4ArmState(self.pAction.seq.seq[i].armTarget.lArm)
                if (not foundSolutionR) or (not foundSolutionL):
                    return False
            
            if (self.pAction.seq.seq[i].type == ActionStep.ARM_TRAJECTORY):
                for j in range(len(self.pAction.seq.seq[i].armTrajectory.timing)):
                    self.pAction.seq.seq[i].armTrajectory.rArm[j], foundSolutionR = self.solveIK4ArmState(self.pAction.seq.seq[i].armTrajectory.rArm[j])
                    self.pAction.seq.seq[i].armTrajectory.lArm[j], foundSolutionL = self.solveIK4ArmState(self.pAction.seq.seq[i].armTrajectory.lArm[j])
                    if (not foundSolutionR) or (not foundSolutionL):
                        return False
        return True

    def solveIK4ArmState(self, armState):
        # We need to find IK only if the frame is relative to an object
        if (armState.refFrame == ArmState.OBJECT):
            solvedArmState = ArmState()
            targetPose = World.transform(armState.ee_pose, '/task_object', '/base_link')
            targetJoints = self.arms[0].getIKForEEPose(targetPose, armState.joint_pose)
            if (targetJoints == None):
                rospy.logerr('Could not find IK for R arm relative pose of action step')
                return solvedArmState, False
            else:
                solvedArmState.refFrame = ArmState.ROBOT_BASE
                solvedArmState.ee_pose = Pose(targetPose.position, targetPose.orientation)
                solvedArmState.joint_pose = targetJoints
                return solvedArmState, True
        else:
            return armState, True
    
    def isConditionMet(self, cond):
        # TODO
        return True
    
    def executeProgrammedAction(self):
        '''
        Function to replay the demonstrated two-arm skill of type ProgrammedAction
        '''
        
        self.executionStatus = ExecutionStatus.EXECUTING
        # Check if the very first precondition is met
        aStep = self.pAction.getStep(0)
        if (not self.isConditionMet(aStep.preCond)):
            rospy.logwarn('First precond is not met, first make sure the robot is ready to execute action (hand object or free hands).')
            self.executionStatus = ExecutionStatus.CONDITION_ERROR
        else:
            # Check that all parts of the action are reachable
            if (not self.solveIK4ProgrammedAction()):
                rospy.logwarn('Problems in finding IK solutions...')
                self.executionStatus = ExecutionStatus.NO_IK
            else:
                self.setArmMode(0, ArmMode.HOLD)
                self.setArmMode(1, ArmMode.HOLD)

                # Go over steps of the action
                for i in range(self.pAction.nFrames()):
                    
                    rospy.loginfo('Executing step ' + str(i))
                    aStep = self.pAction.getStep(i)

                    # Check that preconditions are met
                    if (self.isConditionMet(aStep.preCond)):
                    
                        # For each step check step type
                        # If arm target action
                        if (aStep.type == ActionStep.ARM_TARGET):
                            rospy.loginfo('Will perform arm target action step.')
                            
                            if (not self.moveToTarget(aStep.armTarget.rArm, aStep.armTarget.lArm)):
                                self.executionStatus = ExecutionStatus.OBSTRUCTED
                                break
        
                        elif (aStep.type == ActionStep.ARM_TRAJECTORY):

                            rospy.loginfo('Will perform arm trajectory action step.')

                            # First move to the start frame
                            if (not self.moveToTarget(aStep.armTrajectory.rArm[0], aStep.armTrajectory.lArm[0])):
                                self.executionStatus = ExecutionStatus.OBSTRUCTED
                                break

                            #  Then execute the trajectory
                            self.arms[0].executeJointTrajectory(aStep.armTrajectory.rArm, aStep.armTrajectory.timing)
                            self.arms[1].executeJointTrajectory(aStep.armTrajectory.lArm, aStep.armTrajectory.timing)
            
                            # Wait until both arms complete the trajectory
                            while((self.arms[0].isExecutingTrajectory() and aStep.armTrajectory.rRefFrame != ArmState.NOT_MOVING) 
                                  or (self.arms[1].isExecutingTrajectory() and aStep.armTrajectory.lRefFrame != ArmState.NOT_MOVING)):
                                time.sleep(0.01)
                            rospy.loginfo('Trajectory complete.')
                
                            # Verify that both arms succeeded
                            if ((not self.arms[0].isExecutionSuccessful()) or (not self.arms[1].isExecutionSuccessful())):
                                rospy.logwarn('Aborting execution because arms failed to follow trajectory.')
                                self.executionStatus = ExecutionStatus.OBSTRUCTED
                                break
                            
                        # If hand action do it for both sides
                        elif (aStep.type == ActionStep.GRIPPER_TARGET):
                            
                            rospy.loginfo('Will perform gripper action step.')
                            
                            if (aStep.gripperTarget.rGripper.state != self.arms[0].getGripperState()):
                                self.arms[0].executeGripperAction(aStep.gripperTarget.rGripper.state)
                                Response.performGazeAction(GazeGoal.FOLLOW_RIGHT_EE)
                            
                            if (aStep.gripperTarget.lGripper.state != self.arms[1].getGripperState()):
                                self.arms[1].executeGripperAction(aStep.gripperTarget.rGripper.state)
                                Response.performGazeAction(GazeGoal.FOLLOW_LEFT_EE)
        
                            # Wait for grippers to be done
                            while(self.arms[0].isGripperMoving() or self.arms[1].isGripperMoving()):
                                time.sleep(0.01)
                            rospy.loginfo('Hands done moving.')
                            
                            # Verify that both grippers succeeded
                            if ((not self.arms[0].isGripperAtGoal()) or (not self.arms[1].isGripperAtGoal())):
                                rospy.logwarn('Hand(s) did not fully close or open!')
                    
                        # Check that postconditions are met
                        if (self.isConditionMet(aStep.preCond)):
                            rospy.loginfo('Post-conditions of the action are met.')
                        else:
                            rospy.logwarn('Post-conditions of action step ' + str(i) + ' are not satisfied. Will abort execution.')
                            self.executionStatus = ExecutionStatus.PREEMPTED
                            break
                    else:
                        rospy.logwarn('Preconditions of action step ' + str(i) + ' are not satisfied. Will abort execution.')
                        self.executionStatus = ExecutionStatus.PREEMPTED
                        break
        
                    if (self.preempt):
                        rospy.logwarn('Execution preempted by user.')
                        self.executionStatus = ExecutionStatus.PREEMPTED
                        break
                    
                    rospy.loginfo('Step ' + str(i) + ' of action is complete.')

            self.arms[0].updateGripperState()
            self.arms[1].updateGripperState()
            self.arms[0].resetMovementHistory()
            self.arms[1].resetMovementHistory()

            if self.executionStatus == ExecutionStatus.EXECUTING:
                self.executionStatus = ExecutionStatus.SUCCEEDED
                rospy.loginfo('Skill execution has succeeded.')
                
                
                
    def moveToTarget(self, rArm, lArm):
        timeToPoseR = None
        timeToPoseL = None

        #  Check if no action for both sides
        if (rArm.refFrame == ArmState.NOT_MOVING):
            rospy.logwarn('Right arm will not move.')
        #  Determine time to target
        else:
            timeToPoseR = Arms.getDurationBetweenPoses(self.arms[0].getEndEffectorState(), rArm.ee_pose)
            rospy.loginfo('Duration until next frame for R arm ' + str(timeToPoseR))
        
        if (lArm.refFrame == ArmState.NOT_MOVING):
            rospy.logwarn('Left arm will not move.')
        else:
            timeToPoseL = Arms.getDurationBetweenPoses(self.arms[1].getEndEffectorState(), lArm.ee_pose)
            rospy.loginfo('Duration until next frame for L arm ' + str(timeToPoseL))

        #  If both arms are moving adjust velocities and look at the most moving arm
        isMovingR = (timeToPoseR != None)
        isMovingL = (timeToPoseL != None)
        if (not isMovingR):
            Response.lookAtPoint(lArm.ee_pose.position)
        elif (not isMovingL):
            Response.lookAtPoint(rArm.ee_pose.position)
        else:
            if (timeToPoseR > timeToPoseL):
                timeToPoseL = timeToPoseR
                Response.lookAtPoint(rArm.ee_pose.position)
            else:
                timeToPoseR = timeToPoseL
                Response.lookAtPoint(lArm.ee_pose.position)

        #  Move arms to target
        if (isMovingR):
            self.arms[0].gotoJointKeyframe(rArm.joint_pose, timeToPoseR)
        if (isMovingL):
            self.arms[1].gotoJointKeyframe(lArm.joint_pose, timeToPoseL)        

        # Wait until both arms complete the trajectory
        while(self.arms[0].isExecutingTrajectory() or self.arms[1].isExecutingTrajectory()):
            time.sleep(0.01)
        rospy.loginfo('Arms reached target.')
        # Verify that both arms succeeded
        if ((not self.arms[0].isExecutionSuccessful() and isMovingR) or (not self.arms[1].isExecutionSuccessful() and isMovingL)):
            rospy.logwarn('Aborting execution because arms failed to move to keyframe.')
            return False
        else:
            return True

    def getMostMovingArm(self):
        threshold = 0.02
        if (self.arms[0].getMovement() < threshold and self.arms[1].getMovement() < threshold):
            return -1
        elif (self.arms[0].getMovement() < threshold):
            return 1
        else:
            return 0

    @staticmethod
    def getDurationBetweenPoses(pose0, pose1, velocity=0.2):
        dist = Arms.getDistanceBetweenPoses(pose0, pose1)
        duration = dist/velocity;
        if duration < 0.5:
           duration = 0.5
        return duration

    @staticmethod
    def getDistanceBetweenPoses(pose0, pose1):
        pos0 = array((pose0.position.x, pose0.position.y, pose0.position.z))
        pos1 = array((pose1.position.x, pose1.position.y, pose1.position.z))
        
        rot0 = array((pose0.orientation.x, pose0.orientation.y, pose0.orientation.z, pose0.orientation.w))
        rot1 = array((pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w))

        wPos = 1.0
        wRot = 0.2
        posDist = wPos*norm(pos0-pos1)
        rotDist = wRot*(1 - numpy.dot(rot0, rot1))
        
        if (posDist > rotDist):
            dist = posDist
        else:
            dist = rotDist
        return dist

    def update(self):
        
        self.arms[0].update(self.isExecuting())
        self.arms[1].update(self.isExecuting())
        
        movingArm = self.getMostMovingArm()
        if (movingArm != self.attendedArm and not self.isExecuting()):
            if (movingArm == -1):
                Response.performGazeAction(GazeGoal.LOOK_FORWARD)
            elif (movingArm == 0):
                Response.performGazeAction(GazeGoal.FOLLOW_RIGHT_EE)
            else:
                Response.performGazeAction(GazeGoal.FOLLOW_LEFT_EE)
            self.attendedArm = movingArm
                        
