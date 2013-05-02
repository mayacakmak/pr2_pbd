#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_pbd_gui')
roslib.load_manifest('speech_recognition')
#roslib.load_manifest('speakeasy');
roslib.load_manifest('sound_play');


import os
from subprocess import call

# ROS libraries
import rospy, yaml
from std_msgs.msg import String
#import qt_gui.qt_binding_helper
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame, QGroupBox, QIcon
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
#from speakeasy.msg import SpeakEasyTextToSpeech
from speech_recognition.msg import Command
from sound_play.msg import SoundRequest


class ClickableLabel(QtGui.QLabel):
    def __init__(self, parent, index, clickCallback):
        QtGui.QLabel.__init__(self, parent)
        self.index = index
        self.clickCallback = clickCallback
    
    def mousePressEvent(self, event):
        self.emit(QtCore.SIGNAL('clicked()'), "Label pressed")
        self.clickCallback(self.index)

class ActionIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, clickCallback):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find pr2_pbd_gui').read()
        path = path[0:len(path)-1]
        self.notSelectedIconPath = path + '/icons/actions0.png'
        self.selectedIconPath = path + '/icons/actions1.png'
        self.selected = True
        self.actionIconWidth = 50
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(self.getName())
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignCenter)
    
    def getName(self):
        return 'Action' + str(self.index + 1)
    
    def updateView(self):
        if self.selected:
            pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth, QtCore.Qt.SmoothTransformation))


class StepIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, clickCallback):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find pr2_pbd_gui').read()
        path = path[0:len(path)-1]
        self.notSelectedIconPath = path + '/icons/node0.png'
        self.selectedIconPath = path + '/icons/node1.png'
        self.notSelectedFirstIconPath = path + '/icons/firstnode0.png'
        self.selectedFirstIconPath = path + '/icons/firstnode1.png'
        self.selected = True
        self.iconWidth = 58
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(self.getName())
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignRight)
    
    def getName(self):
        return str(self.index + 1)
    
    def hide(self):
        self.icon.hide()
        self.text.hide()
    
    def show(self):
        self.icon.show()
        self.text.show()
    
    def updateView(self):
        if self.index == 0:
            if self.selected:
                pixmap = QtGui.QPixmap(self.selectedFirstIconPath)
            else:
                pixmap = QtGui.QPixmap(self.notSelectedFirstIconPath)
        else:
            if self.selected:
                pixmap = QtGui.QPixmap(self.selectedIconPath)
            else:
                pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.iconWidth, QtCore.Qt.SmoothTransformation))
        
class PbDGUI(Plugin):

    newCommand = Signal(Command)

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.setObjectName('PbDGUI')
        self._widget = QWidget()
        self.getCommandList()
        self.commandOutput = rospy.Publisher('recognized_command', Command)
        rospy.Subscriber('recognized_command', Command, self.speechCommandReceived)
        #self.speechInput = rospy.Subscriber('speakeasy_text_to_speech_req', SpeakEasyTextToSpeech, self.robotSpeechReceived)
        self.soundInput = rospy.Subscriber('robotsound', SoundRequest, self.robotSoundReceived)
        self.stateInput = rospy.Subscriber('interaction_state', String, self.robotStateReceived)
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.newCommand.connect(self.respondToCommand)
        
        self.commandButtons = dict()
        self.commandButtons[Command.CREATE_NEW_ACTION] = 'New action'
        self.commandButtons[Command.TEST_MICROPHONE] = 'Test microphone'
        self.commandButtons[Command.NEXT_ACTION] = 'Next action'
        self.commandButtons[Command.PREV_ACTION] = 'Prev action'
        self.commandButtons[Command.SAVE_POSE] = 'Save pose'
        
        self.commandButtons[Command.RELAX_RIGHT_ARM] = 'Relax right arm'
        self.commandButtons[Command.RELAX_LEFT_ARM] = 'Relax left arm'
        self.commandButtons[Command.FREEZE_RIGHT_ARM] = 'Freeze right arm'
        self.commandButtons[Command.FREEZE_LEFT_ARM] = 'Freeze left arm'
        
        self.commandButtons[Command.OPEN_RIGHT_HAND] = 'Open right hand'
        self.commandButtons[Command.OPEN_LEFT_HAND] = 'Open left hand'
        self.commandButtons[Command.CLOSE_RIGHT_HAND] = 'Close right hand'
        self.commandButtons[Command.CLOSE_LEFT_HAND] = 'Close left hand'

        self.commandButtons[Command.CLOSE_LEFT_HAND] = 'Close left hand'
        
        self.commandButtons[Command.EXECUTE_ACTION] = 'Execute action'
        self.commandButtons[Command.STOP_EXECUTION] = 'Stop execution'
        self.commandButtons[Command.DELETE_ALL_STEPS] = 'Delete all'
        self.commandButtons[Command.DELETE_LAST_STEP] = 'Delete last'

        self.commandButtons[Command.RECORD_OBJECT_POSE] = 'Save object states'
        
        self.currentAction = -1
        self.currentStep = -1

        allWidgetsBox = QtGui.QVBoxLayout()
        actionBox = QGroupBox('Actions', self._widget)
        self.actionGrid = QtGui.QGridLayout()
        self.actionGrid.setHorizontalSpacing(0)
        for i in range(6):
            self.actionGrid.addItem(QtGui.QSpacerItem(90, 90), 0, i, QtCore.Qt.AlignCenter)
            #self.actionGrid.setColumnMinimumWidth(i,88)
            self.actionGrid.setColumnStretch(i, 0)
        self.actionIcons = dict()
        actionBoxLayout = QtGui.QHBoxLayout()
        actionBoxLayout.addLayout(self.actionGrid)
        actionBox.setLayout(actionBoxLayout)
        
        actionButtonGrid = QtGui.QHBoxLayout()
        btn = QtGui.QPushButton(self.commandButtons[Command.CREATE_NEW_ACTION], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        actionButtonGrid.addWidget(btn)
        
        self.stepsBox = QGroupBox('No actions created yet', self._widget)
        self.stepsGrid = QtGui.QGridLayout()
        self.stepsGrid.setHorizontalSpacing(0)
        self.stepsGrid.setSpacing(0)
        for i in range(8):
            self.stepsGrid.addItem(QtGui.QSpacerItem(60, 60), 0, i, QtCore.Qt.AlignCenter)
            self.stepsGrid.setColumnMinimumWidth(i,20)
            self.stepsGrid.setColumnStretch(i, 0)
        self.stepsGrid.addItem(QtGui.QSpacerItem(60, 60), 0, 8)
        self.stepsGrid.setColumnStretch(8, 1)
        self.actionSteps = dict()
        stepsBoxLayout = QtGui.QHBoxLayout()
        stepsBoxLayout.addLayout(self.stepsGrid)
        self.stepsBox.setLayout(stepsBoxLayout)
        
        stepsButtonGrid = QtGui.QHBoxLayout()
        btn = QtGui.QPushButton(self.commandButtons[Command.SAVE_POSE], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        stepsButtonGrid.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.EXECUTE_ACTION], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        stepsButtonGrid.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.STOP_EXECUTION], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        stepsButtonGrid.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.DELETE_ALL_STEPS], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        stepsButtonGrid.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.DELETE_LAST_STEP], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        stepsButtonGrid.addWidget(btn)

        
        miscButtonGrid = QtGui.QHBoxLayout()
        btn = QtGui.QPushButton(self.commandButtons[Command.TEST_MICROPHONE], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.RECORD_OBJECT_POSE], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid.addWidget(btn)
        miscButtonGrid.addStretch(1)
        
        miscButtonGrid2 = QtGui.QHBoxLayout()
        btn = QtGui.QPushButton(self.commandButtons[Command.RELAX_RIGHT_ARM], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid2.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.RELAX_LEFT_ARM], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid2.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.FREEZE_RIGHT_ARM], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid2.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.FREEZE_LEFT_ARM], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid2.addWidget(btn)
        miscButtonGrid2.addStretch(1)

        miscButtonGrid3 = QtGui.QHBoxLayout()
        btn = QtGui.QPushButton(self.commandButtons[Command.OPEN_RIGHT_HAND], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid3.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.OPEN_LEFT_HAND], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid3.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.CLOSE_RIGHT_HAND], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid3.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.CLOSE_LEFT_HAND], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid3.addWidget(btn)
        miscButtonGrid3.addStretch(1)
        
        miscButtonGrid4 = QtGui.QHBoxLayout()
        btn = QtGui.QPushButton(self.commandButtons[Command.PREV_ACTION], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid4.addWidget(btn)
        btn = QtGui.QPushButton(self.commandButtons[Command.NEXT_ACTION], self._widget)
        btn.clicked.connect(self.commandButtonPressed)
        miscButtonGrid4.addWidget(btn)
        miscButtonGrid4.addWidget(btn)
        miscButtonGrid4.addStretch(1)

        speechGroupBox = QGroupBox('Robot Speech', self._widget)
        speechGroupBox.setObjectName('RobotSpeechGroup')
        speechBox = QtGui.QHBoxLayout()
        self.speechLabel = QtGui.QLabel('Robot has not spoken yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.speechLabel.setPalette(palette)
        speechBox.addWidget(self.speechLabel)
        speechGroupBox.setLayout(speechBox)

        allWidgetsBox.addWidget(actionBox)
        allWidgetsBox.addLayout(actionButtonGrid)
        
        allWidgetsBox.addWidget(self.stepsBox)
        allWidgetsBox.addLayout(stepsButtonGrid)
        
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(miscButtonGrid)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(miscButtonGrid2)
        allWidgetsBox.addLayout(miscButtonGrid3)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(miscButtonGrid4)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addWidget(speechGroupBox)
        allWidgetsBox.addStretch(1)
        
        # Fix layout and add main widget to the user interface
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('plastique'))
        vAllBox = QtGui.QVBoxLayout()
        vAllBox.addLayout(allWidgetsBox)
        vAllBox.addStretch(1)
        hAllBox = QtGui.QHBoxLayout()
        hAllBox.addLayout(vAllBox)   
        hAllBox.addStretch(1)
        self._widget.setObjectName('PbDGUI')
        self._widget.setLayout(hAllBox)
        context.add_widget(self._widget)

        isReload = rospy.get_param('/pr2_pbd_gui/isReload')
        if isReload:
            self.loadStateForExperiment()

    def loadStateForExperiment(self):
        expNum = rospy.get_param('/pr2_pbd_gui/experimentNumber')
        dataDir = rospy.get_param('/pr2_pbd_gui/dataRoot') + '/data/experiment' + str(expNum) + '/'
        f = open(dataDir + 'experimentState.yaml', 'r')
        expState = yaml.load(f)
        nProgrammedActions = expState['nProgrammedActions']
        
        for i in range(nProgrammedActions):
            self.createNewAction()

        currentAction = expState['currentProgrammedActionIndex']
        self.actionPressed(currentAction-1, False)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.commandOutput.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a dialog to offer the user a set of configuration

    def nActions(self):
        return len(self.actionIcons.keys())

    def createNewAction(self):
        nColumns = 6
        actionIndex = self.nActions()
        if (actionIndex > 0):
            self.hideCurrentAction()
        for key in self.actionIcons.keys():
             self.actionIcons[key].selected = False
             self.actionIcons[key].updateView()
        actIcon = ActionIcon(self._widget, actionIndex, self.actionPressed)
        self.actionGrid.addLayout(actIcon, int(actionIndex/nColumns), actionIndex%nColumns)
        self.actionIcons[actionIndex] = actIcon
        self.actionSteps[actionIndex] = []
        self.currentAction = actionIndex
        self.stepsBox.setTitle('Steps for Action ' + str(self.currentAction+1))

    def stepPressed(self, stepIndex):
        print 'pressed step ', stepIndex
        nSteps = len(self.actionSteps[self.currentAction])
        for i in range(nSteps):
            if (stepIndex == i):
                self.actionSteps[self.currentAction][i].selected = True
            else:
                self.actionSteps[self.currentAction][i].selected = False
            self.actionSteps[self.currentAction][i].updateView()
        self.currentStep = stepIndex
        
    def actionPressed(self, actionIndex, isPublish=True):
        print 'pressed Action ', str(actionIndex+1)
        self.hideCurrentAction()
        for i in range(len(self.actionIcons.keys())):
            key = self.actionIcons.keys()[i]
            if key == actionIndex:
                 self.actionIcons[key].selected = True
            else:
                 self.actionIcons[key].selected = False
            self.actionIcons[key].updateView()
        self.currentAction = actionIndex
        self.stepsBox.setTitle('Steps for Action ' + str(self.currentAction+1))
        if isPublish:
            self.commandOutput.publish(Command('SWITCH_TO_ACTION' + str(actionIndex+1)))
        self.showCurrentAction()
        
    def commandButtonPressed(self):
        clickedButtonName = self._widget.sender().text()
        for key in self.commandButtons.keys():
            if (self.commandButtons[key] == clickedButtonName):
                qWarning('Sending speech command: '+ key)
                command = Command()
                command.command = key
                self.commandOutput.publish(command)
        
    def robotSoundReceived(self, soundReq):
        if (soundReq.command == SoundRequest.SAY):
            qWarning('Robot said: ' + soundReq.arg)
            self.speechLabel.setText('Robot sound: ' + soundReq.arg)
    
    def respondToCommand(self, command):
        qWarning('Received signal:' + command.command)
        nActions = len(self.actionIcons.keys())
        if command.command == Command.CREATE_NEW_ACTION:
            self.createNewAction()
        
        elif command.command == Command.NEXT_ACTION:
            if (self.nActions() > 0):
                if (self.currentAction < self.nActions()-1):
                    self.actionPressed(self.currentAction+1, False)
                else:
                    qWarning('No actions after Action ' + str(self.currentAction+1))
            else:
                qWarning('No actions created yet.')

        elif command.command == Command.PREV_ACTION:
            if (self.nActions() > 0):
                if (self.currentAction > 0):
                    self.actionPressed(self.currentAction-1, False)
                else:
                    qWarning('No actions before Action ' + str(self.currentAction+1))
            else:
                qWarning('No actions created yet.')
                
        elif command.command == Command.SAVE_POSE:
            if (self.nActions() > 0):
                self.savePose()
            else:
                qWarning('No actions created yet.')
            
    def savePose(self):
        nColumns = 9
        stepIndex = len(self.actionSteps[self.currentAction])
        stepIcon = StepIcon(self._widget, stepIndex, self.stepPressed)

        for i in range(stepIndex):
             self.actionSteps[self.currentAction][i].selected = False
             self.actionSteps[self.currentAction][i].updateView()

        self.stepsGrid.addLayout(stepIcon, int(stepIndex/nColumns), stepIndex%nColumns, QtCore.Qt.AlignCenter)
        self.actionSteps[self.currentAction].append(stepIcon)
        self.currentStep = stepIndex
    
    def hideCurrentAction(self):
        nSteps = len(self.actionSteps[self.currentAction])
        for i in range(nSteps):
            self.actionSteps[self.currentAction][i].hide()

    def showCurrentAction(self):
        nSteps = len(self.actionSteps[self.currentAction])
        for i in range(nSteps):
            self.actionSteps[self.currentAction][i].show()

    def speechCommandReceived(self, command):
        qWarning('Received speech command:' + command.command)
        self.newCommand.emit(command)
        
    def robotSpeechReceived(self, speech):
        qWarning('Robot said: ' + speech.text)
        self.speechLabel.setText('Robot said: ' + speech.text)
        
    def robotStateReceived(self, speech):
        qWarning('Robot state: ' + speech.data)
        #self.stateLabel.setText(speech.data)

    def getCommandList(self):
        path = os.popen('rospack find speech_recognition').read()
        path = path[0:len(path)-1]
        msgFile = open(path + '/msg/Command.msg', 'r')
        msgLine = msgFile.readline()
        self.commandList = []
        while (msgLine != ''):
            if (msgLine.find('=') != -1):
                lineParts = msgLine.split("=")
                commandStr = lineParts[len(lineParts)-1]
                while(commandStr[0] == ' '):
                    commandStr = commandStr[1:(len(commandStr))]
                while(commandStr[len(commandStr)-1] == ' ' or commandStr[len(commandStr)-1] == '\n'):
                    commandStr = commandStr[0:(len(commandStr)-1)]
                #commandStr = commandStr[1:(len(commandStr)-1)]
                if(commandStr != 'unrecognized'):
                    self.commandList.append(commandStr);
            msgLine = msgFile.readline()
        msgFile.close()
        
        
        