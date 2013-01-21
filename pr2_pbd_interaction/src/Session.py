import roslib
from ProgrammedAction import *
import os, sys, yaml

class Session:
    "This class holds and maintains experimental data"
    def __init__(self, objectList, isDebug=False):
        self.reloadState = False
        if (isDebug):
            self.expNum = 0
            self.dataDir = self.getDataDir(self.expNum)
            if (not os.path.exists(self.dataDir)):
                os.mkdir(self.dataDir)
        else:
            self.getParticipantID()
        rospy.set_param('data_directory', self.dataDir)
        
        self.allProgrammedActions = dict()
        self.currentProgrammedActionIndex = 0
        self.backupProgrammedAction = None

        if (self.reloadState):
            self.loadStateForSession(objectList)
            rospy.loginfo("Session state loaded.")
        
    def getParticipantID(self):
        self.expNum = None
        while (self.expNum == None):
            try:
                self.expNum = int(raw_input('Please enter participant ID:'))
            except ValueError:
                rospy.logerr("Participant ID needs to be a number")
                
            self.dataDir = self.getDataDir(self.expNum)
            if (not os.path.exists(self.dataDir)):
                os.mkdir(self.dataDir)
            else:
                rospy.logwarn('A directory for this participant ID already exists: '+ self.dataDir)
                overwrite = raw_input('Do you want to overwrite? Type r to reload the last state of the experiment. [y/n/r]')
                if (overwrite == 'y'):
                    continue
                elif (overwrite == 'n'):
                    self.expNum = None
                elif (overwrite == 'r'):
                    self.reloadState = True
                else:
                    rospy.logerr('Invalid response, try again.')
    
    def getDataDir(self, expNum):
        return rospy.get_param('/pr2_pbd_interaction/dataRoot') + '/data/experiment' + str(expNum) + '/'
    
    def saveSessionState(self, saveActions=True):
        expState = dict()
        expState['nProgrammedActions'] = self.nProgrammedActions()
        expState['currentProgrammedActionIndex'] = self.currentProgrammedActionIndex
        
        f = open(self.dataDir + 'experimentState.yaml', 'w')
        f.write(yaml.dump(expState))
        f.close()

        if (saveActions):
            for i in range(self.nProgrammedActions()):
                self.allProgrammedActions[i].save(self.dataDir)
        
    def loadStateForSession(self, objectList):
        f = open(self.dataDir + 'experimentState.yaml', 'r')
        expState = yaml.load(f)
        nProgrammedActions = expState['nProgrammedActions']
        for i in range(nProgrammedActions):
            self.allProgrammedActions.update({(i+1): ProgrammedAction(i+1)})
            self.allProgrammedActions[(i+1)].load(self.dataDir)
        self.currentProgrammedActionIndex = expState['currentProgrammedActionIndex']
        self.allProgrammedActions[self.currentProgrammedActionIndex].initializeVisualization(objectList)

    def newProgrammedAction(self):
        if (self.nProgrammedActions() > 0):
            self.getProgrammedAction().resetVisualization()
        self.currentProgrammedActionIndex += 1
        self.allProgrammedActions.update({self.currentProgrammedActionIndex: ProgrammedAction(self.currentProgrammedActionIndex)})

    def nProgrammedActions(self):
        return len(self.allProgrammedActions)
    
    def getProgrammedAction(self):
        return self.allProgrammedActions[self.currentProgrammedActionIndex]
    
    def getProgrammedActionName(self):
        return self.allProgrammedActions[self.currentProgrammedActionIndex].getName()

    def clearProgrammedAction(self):
        if (self.nProgrammedActions() > 0):
            self.allProgrammedActions[self.currentProgrammedActionIndex].clear()
        else:
            rospy.logwarn('No skills created yet.')
            
    def undoClearProgrammedAction(self):
        if (self.nProgrammedActions() > 0):
            self.allProgrammedActions[self.currentProgrammedActionIndex].undoClear()
        else:
            rospy.logwarn('No skills created yet.')

    def saveProgrammedAction(self):
        if (self.nProgrammedActions() > 0):
            self.allProgrammedActions[self.currentProgrammedActionIndex].save(self.dataDir)
            self.saveSessionState(saveActions=False)
        else:
            rospy.logwarn('No skills created yet.')
    
    def addStepToProgrammedAction(self, step, objectList):
        if (self.nProgrammedActions() > 0):
            self.allProgrammedActions[self.currentProgrammedActionIndex].addActionStep(step, objectList)
        else:
            rospy.logwarn('No skills created yet.')

    def getLastStepOfProgrammedAction(self):
        if (self.nProgrammedActions() > 0 and self.nFrames() > 0):
            return self.allProgrammedActions[self.currentProgrammedActionIndex].getLastStep()
        else:
            rospy.logwarn('No skills created yet. Or no steps yet.')

    def deleteLastStep(self):
        if (self.nProgrammedActions() > 0):
            self.allProgrammedActions[self.currentProgrammedActionIndex].deleteLastStep()
        else:
            rospy.logwarn('No skills created yet.')

    def resumeDeletedPose(self):
        if (self.nProgrammedActions() > 0):
            self.allProgrammedActions[self.currentProgrammedActionIndex].resumeDeletedPose()
        else:
            rospy.logwarn('No skills created yet.')

    def nextProgrammedAction(self, objectList):
        if (self.nProgrammedActions() > 0):
            if (self.currentProgrammedActionIndex < self.nProgrammedActions()):
                self.getProgrammedAction().resetVisualization()
                self.currentProgrammedActionIndex += 1
                self.getProgrammedAction().initializeVisualization(objectList)
                return True
            else: 
                return False
        else:
            rospy.logwarn('No skills created yet.')

    def previousProgrammedAction(self, objectList):
        if (self.nProgrammedActions() > 0):
            if (self.currentProgrammedActionIndex > 1):
                self.getProgrammedAction().resetVisualization()
                self.currentProgrammedActionIndex -= 1
                self.getProgrammedAction().initializeVisualization(objectList)
                return True
            else: 
                return False
        else:
            rospy.logwarn('No skills created yet.')
    
    def nFrames(self):
        if (self.nProgrammedActions() > 0):
            return self.allProgrammedActions[self.currentProgrammedActionIndex].nFrames()
        else:
            rospy.logwarn('No skills created yet.')
            return 'NA'

    def getCurrentStatus(self):
        statusStr = ''
        statusStr += 'Total number of skills:' + str(self.nProgrammedActions()) + '\n'
        statusStr += 'Current skill: ProgrammedAction' + str(self.currentProgrammedActionIndex) + '\n'
        statusStr += 'Number of poses in skill:' + str(self.nFrames())
        return statusStr


