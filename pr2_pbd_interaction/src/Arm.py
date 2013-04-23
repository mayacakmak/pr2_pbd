# Dependencies
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('tf')
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('pr2_gripper_sensor_msgs')
roslib.load_manifest('pr2_pbd_interaction')
roslib.load_manifest('arm_navigation_msgs')

# Generic libraries
from scipy.ndimage.filters import *
import time, sys, threading
from numpy import *
from numpy.linalg import norm

# ROS libraries
import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from tf import TransformListener
from pr2_mechanism_msgs.srv import SwitchController
from trajectory_msgs.msg import *
from kinematics_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from arm_navigation_msgs.srv import *

# Local
from pr2_pbd_interaction.msg import *
from World import *

# Enums
class ArmSide:
    RIGHT = 0
    LEFT = 1

class Arm:
    def __init__(self, armIndex):
    
        self.armIndex = armIndex
        if (armIndex == ArmSide.RIGHT):
            self.side = 'right'
        elif (armIndex == ArmSide.LEFT):
            self.side = 'left'
        self.sidePrefix = self.side[0]
        
        self.armMode = ArmMode.HOLD
        self.gripperState = None

        self.gripperJointName = self.sidePrefix + '_gripper_joint';
        self.endEffector = self.sidePrefix + '_wrist_roll_link'
        self.joints = [self.sidePrefix + '_shoulder_pan_joint', self.sidePrefix + '_shoulder_lift_joint',
                       self.sidePrefix + '_upper_arm_roll_joint', self.sidePrefix + '_elbow_flex_joint', 
                       self.sidePrefix + '_forearm_roll_joint', self.sidePrefix + '_wrist_flex_joint', self.sidePrefix + '_wrist_roll_joint']
            
        self.allJointNames = []    
        self.allJointPositions = []

        self.lastEEPose = None
        self.armMovementBufferSize = 40
        self.lastUnstableTime = rospy.Time.now()
        self.armMovement = []

        rospy.loginfo('Initializing ' + self.side + ' arm.')

        self.switchController = 'pr2_controller_manager/switch_controller'
        rospy.wait_for_service(self.switchController)
        self.controllerService = rospy.ServiceProxy(self.switchController, SwitchController)
        rospy.loginfo('Got response form the switch controller for ' + self.side + ' arm.'); 

        # Create a trajectory action client.
        self.trajectoryActionClient = actionlib.SimpleActionClient(self.sidePrefix + '_arm_controller/joint_trajectory_action'
                                                         , JointTrajectoryAction)
        self.trajectoryActionClient.wait_for_server()
        rospy.loginfo('Got response form trajectory action server for ' + self.side + ' arm.'); 

        ## Next setup communication related stuff.
        self.setupFK()
        self.setupIK()
        self.setupGripper()
        
        filterServiceName = '/trajectory_filter/filter_trajectory' #'/trajectory_filter_server/filter_trajectory_with_constraints' #/trajectory_filter_unnormalizer/filter_trajectory'
        rospy.wait_for_service(filterServiceName)
        self.filterServiceP = rospy.ServiceProxy(filterServiceName, FilterJointTrajectory)
        rospy.loginfo('Filtering service has responded for '+self.side+' arm.')

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self.jointStatesCallback)
         
    def setupFK(self):
        fkInfoSrvName = 'pr2_' + self.side + '_arm_kinematics/get_fk_solver_info'
        fkSrvName = 'pr2_'+self.side +'_arm_kinematics/get_fk'
        rospy.wait_for_service(fkInfoSrvName)
        self.fkServiceInfoP = rospy.ServiceProxy(fkInfoSrvName, GetKinematicSolverInfo)
        solver_info = self.fkServiceInfoP()
        rospy.loginfo('FK info service has responded for '+self.side+' arm.')
        rospy.wait_for_service(fkSrvName)
        self.fkServiceP = rospy.ServiceProxy(fkSrvName, GetPositionFK, persistent=True)
        self.fkrequest = GetPositionFKRequest()
        rospy.loginfo('FK service has responded for '+self.side+' arm.')
        self.fkrequest.header.frame_id = "torso_lift_link"
        self.fkrequest.fk_link_names = [self.endEffector]
    
    def setupIK(self):
        ikInfoSrvName = 'pr2_'+ self.side +'_arm_kinematics/get_ik_solver_info'
        ikSrvName = 'pr2_'+ self.side +'_arm_kinematics/get_ik'
        rospy.wait_for_service(ikInfoSrvName)
        self.ikServiceInfoP = rospy.ServiceProxy(ikInfoSrvName, GetKinematicSolverInfo)
        solver_info = self.ikServiceInfoP()
        rospy.loginfo('IK info service has responded for '+self.side+' arm.')
        rospy.wait_for_service(ikSrvName)
        self.ikServiceP = rospy.ServiceProxy(ikSrvName, GetPositionIK, persistent=True)
        rospy.loginfo('IK service has responded for '+self.side+' arm.')
        self.ikrequest = GetPositionIKRequest()
        self.ikrequest.timeout = rospy.Duration(4.0)
        self.ikjoints = solver_info.kinematic_solver_info.joint_names
        self.iklimits = solver_info.kinematic_solver_info.limits
        iklinks = solver_info.kinematic_solver_info.link_names

        self.ikrequest.ik_request.ik_link_name = iklinks[0]
        self.ikrequest.ik_request.pose_stamped.header.frame_id = 'base_link'
        self.ikrequest.ik_request.ik_seed_state.joint_state.name = self.ikjoints
        self.ikrequest.ik_request.ik_seed_state.joint_state.position = [0]*len(self.ikjoints)
        
    def setupGripper(self):
        self.gripperClient = actionlib.SimpleActionClient(self.sidePrefix+'_gripper_controller/gripper_action',
                                                          Pr2GripperCommandAction);
        self.gripperClient.wait_for_server()

    def getEndEffectorState(self, refName='base_link'):
        try:
            t = World.tfListener.getLatestCommonTime(refName, self.endEffector)
            (position, orientation) = World.tfListener.lookupTransform(refName, self.endEffector, t)
            eePose = Pose()
            eePose.position = Point(position[0], position[1], position[2])
            eePose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
            return eePose
        except:
            rospy.logwarn('Something wrong with transform request.')
            return None

    def jointStatesCallback(self, msg):
    #Callback function that saves the joint positions when a joint_states message is received
        self.lock.acquire()
        self.allJointNames = msg.name
        self.allJointPositions = msg.position
        self.lastJointStateTime = msg.header.stamp
        self.lock.release()
           
    def getJointPositions(self, joint_names = []):
        if joint_names == []:
            joint_names = self.joints

        if self.allJointNames == []:
            rospy.logerr("No robot_state messages received!\n")
            return []
        
        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.allJointNames:
                index = self.allJointNames.index(joint_name)
                position = self.allJointPositions[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
        self.lock.release()
        return positions

    def getEndEffectorIK(self, pose, seed = []):
        # Desired EE pose for the IK request 
        self.ikrequest.ik_request.pose_stamped.pose = pose

        # Seed for searching the IK solution
        if seed == [] or seed == None:
            seed = []
            for i in range(0, len(self.ikjoints)):
                seed.append((self.iklimits[i].min_position + self.iklimits[i].max_position)/2.0)
        self.ikrequest.ik_request.ik_seed_state.joint_state.position = seed
        
        try:
	    rospy.loginfo('Sending IK request.')
            response = self.ikServiceP(self.ikrequest)
            if(response.error_code.val == response.error_code.SUCCESS):
                return response.solution.joint_state.position
            else :
                return None
        except rospy.ServiceException, e:
            rospy.logerr('Exception while getting the IK solution.')
            return None	
    
    def setMode(self, mode):
        armControllerName = self.sidePrefix + '_arm_controller'
        if mode == ArmMode.RELEASE:
            start_controllers = []
            stop_controllers  = [armControllerName]
            rospy.loginfo('Switching ' + str(self.side) + ' arm to the kinesthetic mode')
        elif mode == ArmMode.HOLD:
            start_controllers = [armControllerName]
            stop_controllers  = []
            rospy.loginfo('Switching ' + str(self.side) + ' to the Joint-control mode') 
        else:
            rospy.logwarn('Unknown mode ' + str(mode) + '. Keeping the current mode.')
            return

        strictness = 1
        try:
            resp1 = self.controllerService(start_controllers, stop_controllers, strictness)
            self.armMode = mode
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: " + str(e))

    def getMode(self):
        return self.armMode

    def gripperCmd(self, pos = 0.08, eff = 30.0, wait=False):
        grippercmd = Pr2GripperCommandGoal()
        grippercmd.command.position = pos
        grippercmd.command.max_effort = eff
        self.gripperClient.send_goal(grippercmd)
        if wait:
            self.gripperClient.wait_for_result(rospy.Duration(10.0))

    def isGripperMoving(self):
        return (self.gripperClient.get_state() == GoalStatus.ACTIVE or
                self.gripperClient.get_state() == GoalStatus.PENDING)

    def isGripperAtGoal(self):
        return (self.gripperClient.get_state() == GoalStatus.SUCCEEDED)

    def getGripperState(self):
        return self.gripperState

    def updateGripperState(self, jointName=None):
        if (jointName == None):
            jointName = self.gripperJointName
        gripPos = self.getJointPositions([jointName])
        if gripPos != []:
            if gripPos[0] > 0.078:
                self.gripperState = GripperState.OPEN
            else:
                self.gripperState = GripperState.CLOSED
        else:
            rospy.logwarn('Could not update the gripper state.')
        
    def openGripper(self, pos = 0.08, eff = 30.0, wait=False):
        self.gripperCmd( pos, eff, wait)
        self.gripperState = GripperState.OPEN
        
    def closeGripper(self, pos = 0.0, eff = 30.0, wait=False):
        self.gripperCmd(pos, eff, wait)
        self.gripperState = GripperState.CLOSED
    
    def executeGripperAction(self, targetState):
        if (targetState == GripperState.CLOSED):
            self.closeGripper();
        elif (targetState == GripperState.OPEN):
            self.openGripper()

    def executeJointTrajectory(self, armTraj, timing):
        
        # Do filtering on the trajectory to fix the velocities
        trajectory = JointTrajectory()
        
        # Initialize the server
        # When to start the trajectory: 0.1 seconds from now
        trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trajectory.joint_names = self.joints

        ## Add all frames of the trajectory as way points
        for i in range(len(timing)):
            positions  = armTraj[i].joint_pose
            velocities = [0]*len(positions)
            # Add frames to the trajectory
            trajectory.points.append(JointTrajectoryPoint(positions = positions, velocities = velocities, time_from_start = timing[i]))

        output = self.filterServiceP(trajectory=trajectory, allowed_time=rospy.Duration.from_sec(20))
        rospy.loginfo('Trajectory for arm ' + str(self.armIndex) + ' has been filtered.')
        trajGoal = JointTrajectoryGoal()
        # TODO: check output.error_code
        trajGoal.trajectory = output.trajectory
        trajGoal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trajGoal.trajectory.joint_names = self.joints
        
        # Client sends the goal to the Server
        self.trajectoryActionClient.send_goal(trajGoal)

    def gotoJointKeyframe(self, jointKeyframe, timeToKeyframe):
        # Setup the goal
        trajGoal = JointTrajectoryGoal()
        trajGoal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trajGoal.trajectory.joint_names = self.joints
 
        velocities = [0]*len(jointKeyframe)
        trajGoal.trajectory.points.append(JointTrajectoryPoint(positions=jointKeyframe, 
                        velocities=velocities, time_from_start=rospy.Duration(timeToKeyframe)))

        # Client sends the goal to the Server
        self.trajectoryActionClient.send_goal(trajGoal)
        
    def getTrajectoryStatus(self):
        return self.trajectoryActionClient.get_state()

    def isExecutingTrajectory(self):
        return (self.trajectoryActionClient.get_state() == GoalStatus.ACTIVE 
                or self.trajectoryActionClient.get_state() == GoalStatus.PENDING)

    def isExecutionSuccessful(self):
        return (self.trajectoryActionClient.get_state() == GoalStatus.SUCCEEDED)

    def getIKForEEPose(self, eePose, seed):
        jConf = self.getEndEffectorIK(eePose, seed)
        ## If our seed did not work, try once again with the default seed
        if jConf == None:
            rospy.logwarn('Could not find IK solution with preferred seed, will try default seed.')
            jConf = self.getEndEffectorIK(eePose)

        if jConf == None:
            rospy.logwarn('IK out of bounds, will use the seed directly.')
        else:
            rollover = array((array(jConf)-array(seed))/pi, int)
            jConf -= ((rollover+(sign(rollover)+1)/2)/2)*2*pi

        return jConf
            
        ## Add a delay to compensate for going to the first frame
#        currentEE = self.getEndEffectorState()
#        if currentEE == None:
#            delay = rospy.Duration(2.0)
#        else:
#            delay = ArmTrajectory.getDurationBetweenPoses(eeTraj.trajectory[0].abs_ee_pose, currentEE)
    
    @staticmethod
    def eeDistance(poseA, poseB):
        if poseA == [] or poseB == []:
            return 0.0
        else:
            dist = norm(array([poseA.position.x,poseA.position.y,poseA.position.z]) - 
                        array([poseB.position.x,poseB.position.y,poseB.position.z]))
            if dist < 0.0001:
                dist = 0
            return dist
    
    def resetMovementHistory(self):
        self.lastUnstableTime = rospy.Time.now()
        self.armMovement = []

    def getMovement(self):
        return sum(self.armMovement)
    
    def recordArmMovement(self, reading):
        self.armMovement = [reading] + self.armMovement
        if (len(self.armMovement) > self.armMovementBufferSize):
            self.armMovement = self.armMovement[0:self.armMovementBufferSize]
    
    def hasArmMovedWhileHolding(self):
        threshold = 0.02
        if (self.getMode() == ArmMode.HOLD and len(self.armMovement) == self.armMovementBufferSize):
            if (self.getMovement() > threshold):
                return True
        return False

    def hasArmBeenStableWhileReleased(self):
        movementThreshold = 0.02
        timeThreshold = rospy.Duration(5.0)
        isArmStable = (self.getMovement() < movementThreshold)
        if (not isArmStable or self.getMode()==ArmMode.HOLD):
            self.lastUnstableTime = rospy.Time.now()
            return False
        else:
            if (rospy.Time.now() - self.lastUnstableTime) > timeThreshold:
                return True
            else:
                return False

    def update(self, isExecuting):
        eePose = self.getEndEffectorState()
        if (eePose != None and self.lastEEPose != None):
            self.recordArmMovement(Arm.eeDistance(eePose, self.lastEEPose))
        self.lastEEPose = eePose
        
#        if (not isExecuting):
#            if (self.hasArmMovedWhileHolding()):
#                rospy.loginfo('Automatically releasing arm.')
#                self.setMode(ArmMode.RELEASE)
                
#            if (self.hasArmBeenStableWhileReleased()):
#                rospy.loginfo('Automatically holding arm.')
#                self.setMode(ArmMode.HOLD)
    
    
    
