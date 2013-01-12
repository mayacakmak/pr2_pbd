#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_pbd_gui')
roslib.load_manifest('speech_recognition')
roslib.load_manifest('speakeasy');
roslib.load_manifest('sound_play');


import os
from subprocess import call

# ROS libraries
import rospy
from std_msgs.msg import String
#import qt_gui.qt_binding_helper
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame, QGroupBox
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning
from speakeasy.msg import SpeakEasyTextToSpeech
from speech_recognition.msg import Command
from sound_play.msg import SoundRequest

class PbDGUI(Plugin):

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.setObjectName('PbDGUI')
        self._widget = QWidget()
        self.getCommandList()
        self.commandOutput = rospy.Publisher('recognized_command', Command)
        self.speechInput = rospy.Subscriber('speakeasy_text_to_speech_req', SpeakEasyTextToSpeech, self.robotSpeechReceived)
        self.soundInput = rospy.Subscriber('robotsound', SoundRequest, self.robotSoundReceived)
        self.stateInput = rospy.Subscriber('interaction_state', String, self.robotStateReceived)
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))

        # Add buttons for sending speech commands
        commandsGroupBox = QGroupBox('Speech Commands', self._widget)
        commandsGroupBox.setObjectName('CommandsGroup')
        grid = QtGui.QGridLayout()
        nColumns = 4
        nCommands = len(self.commandList)
        for i in range(0, nCommands):
            btn = QtGui.QPushButton(self.commandList[i], self._widget)
            btn.clicked.connect(self.commandButtonPressed)
            grid.addWidget(btn, int(i/nColumns), i%nColumns)        
        
        commandBox = QtGui.QHBoxLayout()
        commandBox.addLayout(grid)
        commandsGroupBox.setLayout(commandBox)

        # Add a display of what the robot says
        speechGroupBox = QGroupBox('Robot Speech', self._widget)
        speechGroupBox.setObjectName('RobotSpeechGroup')
        speechBox = QtGui.QHBoxLayout()
        self.speechLabel = QtGui.QLabel('Robot has not spoken yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.speechLabel.setPalette(palette)
        speechBox.addWidget(self.speechLabel)
        speechGroupBox.setLayout(speechBox)

        # Add a display of what the robot says
        stateGroupBox = QGroupBox('Robot State', self._widget)
        stateGroupBox.setObjectName('RobotStateGroup')
        stateBox = QtGui.QHBoxLayout()
        self.stateLabel = QtGui.QLabel('Robot state not received yet.\n\n\n')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.red)
        self.stateLabel.setPalette(palette)
        stateBox.addWidget(self.stateLabel)
        stateGroupBox.setLayout(stateBox)

        # Add all children widgets into the main widget
        allWidgetsBox = QtGui.QGridLayout()
        allWidgetsBox.addWidget(commandsGroupBox, 0, 0)
        allWidgetsBox.addWidget(speechGroupBox, 1, 0)
        allWidgetsBox.addWidget(stateGroupBox, 2, 0)
        
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

    def commandButtonPressed(self):
        clickedCommand = self._widget.sender()
        qWarning('Sending speech command: '+clickedCommand.text())
        command = Command()
        command.command = clickedCommand.text()
        self.commandOutput.publish(command)
        
    def robotSoundReceived(self, soundReq):
        if (soundReq.command == SoundRequest.SAY):
            qWarning('Robot said: ' + soundReq.arg)
            self.speechLabel.setText('Robot said: ' + soundReq.arg)
        
    def robotSpeechReceived(self, speech):
        qWarning('Robot said: ' + speech.text)
        self.speechLabel.setText('Robot said: ' + speech.text)
        
    def robotStateReceived(self, speech):
        qWarning('Robot state: ' + speech.data)
        self.stateLabel.setText(speech.data)

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
        
        
        