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
		Command.RELAX_RIGHT_ARM, 
		Command.RELAX_LEFT_ARM, 
		Command.OPEN_RIGHT_HAND, 
		Command.OPEN_LEFT_HAND, 
		Command.CLOSE_RIGHT_HAND, 
		Command.CLOSE_LEFT_HAND, 
		Command.STOP_EXECUTION, 
		Command.UNDO, 
		Command.DELETE_ALL_STEPS, 
		Command.DELETE_LAST_STEP, 
		Command.FREEZE_RIGHT_ARM, 
		Command.FREEZE_LEFT_ARM, 
		Command.RECORD_OBJECT_POSE, 
		Command.CREATE_NEW_ACTION, 
		Command.EXECUTE_ACTION, 
		Command.NEXT_ACTION, 
		Command.PREV_ACTION, 
		Command.SAVE_ACTION, 
		Command.EDIT_ACTION, 
		Command.SAVE_POSE, 
		Command.START_RECORDING_MOTION, 
		Command.STOP_RECORDING_MOTION]

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
