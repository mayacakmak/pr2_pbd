#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_pbd_speech_recognition')
import rospy
from std_msgs.msg import String
from pr2_pbd_speech_recognition.msg import Command

class CommandRecognizer:
    def __init__(self):
        rospy.Subscriber('recognizer/output', String, self.receiveSphinxData)
        self.commandOutput = rospy.Publisher('recognized_command', Command)
        self.allCommands = [Command.TEST_MICROPHONE, 
		Command.TAKE_TOOL,
		Command.START_RECORDING,
		Command.STOP_RECORDING,
		Command.REPLAY_DEMONSTRATION,
		Command.RELEASE_TOOL,
		Command.DETECT_SURFACE]

    def receiveSphinxData(self,data):
        recognizedStr = data.data
        recognizedCommand = Command.UNRECOGNIZED
        
        for commandStr in self.allCommands:
            if (recognizedStr == commandStr):
                recognizedCommand = commandStr
                
        rospy.loginfo('Received command:' + recognizedCommand)
        command = Command()
        command.command = recognizedCommand
        self.commandOutput.publish(command)

if __name__ == '__main__':
    rospy.init_node('command_recognizer')
    crec = CommandRecognizer()
    rospy.spin()
