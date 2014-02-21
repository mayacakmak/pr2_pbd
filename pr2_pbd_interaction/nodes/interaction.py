#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_pbd_interaction');

import sys
import signal
import rospy
from Interaction import *

def signal_handler(signal, frame):
    # The following makes sure the state of a user study is saved, so that it can be recovered
    global interaction
    interaction.save_experiment_state()
    print 'Program Terminated!!'
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGQUIT, signal_handler)

if __name__ == "__main__":
    global interaction
    rospy.init_node('pr2_pbd_interaction', anonymous=True)
    interaction = Interaction()
    #rospy.spin()
    while(not rospy.is_shutdown()):
        interaction.update()