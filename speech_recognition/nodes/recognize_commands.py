#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('speech_recognition')
import rospy
rospy.init_node('command_recognizer')
from std_msgs.msg import String
from speech_recognition.msg import Command

class CommandRecognizer:
    def __init__(self):
        rospy.Subscriber('recognizer/output', String, self.receiveSphinxData)
        self.commandOutput = rospy.Publisher('recognized_command', Command)
        self.allCommands = [Command.TEST_MICROPHONE, Command.OPEN_RIGHT_HAND, Command.OPEN_LEFT_HAND,
                            Command.CLOSE_RIGHT_HAND,Command.CLOSE_LEFT_HAND, Command.RELAX_RIGHT_ARM, 
                            Command.RELAX_LEFT_ARM, Command.HOLD_RIGHT_ARM, Command.HOLD_LEFT_ARM, 
                            Command.CREATE_NEW_SKILL, Command.STOP_EXECUTION, Command.SAVE_POSE, 
                            Command.DELETE_LAST_POSE, Command.EXECUTE_SKILL, Command.DELETE_ALL_POSES, 
                            Command.NEXT_SKILL, Command.PREV_SKILL, Command.UNDO]
    
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
    crec = CommandRecognizer()
    rospy.spin()
