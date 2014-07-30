#!/usr/bin/env python
'''Fake AR markers for debugging'''
import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import time

# ROS libraries
import rospy
import numpy
from geometry_msgs.msg import Quaternion, Point, Pose
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

class SimWorld:
    '''Object recognition and localization related stuff'''

    def __init__(self):

        self.marker_publisher = rospy.Publisher('ar_pose_marker',
                                                AlvarMarkers)
        self.table_poses = {
            1: Pose(Point(0.5, 0.1, 0.5), Quaternion(0, 0, 1, 1)),
            2: Pose(Point(0.5, -0.1, 0.5), Quaternion(0, 0, 1, 1)),
            3: Pose(Point(0.7, 0.1, 0.5), Quaternion(0, 0, 1, 1)),
            4: Pose(Point(0.7, -0.1, 0.5), Quaternion(0, 0, 1, 1))
        }

        self.tool_pose = Pose(Point(0.5, 0.0, 1.2), Quaternion(0, 1, 0, 1))

    def get_markers(self):
        m_all = AlvarMarkers()

        for k in self.table_poses.keys():
            m = AlvarMarker()
            m.id = k
            m.pose.pose = self.table_poses[k]
            m_all.markers.append(m)

        tool_id = 99
        if rospy.has_param('cleaning_tool_id'):
            tool_id = rospy.get_param('cleaning_tool_id')
            
        mt = AlvarMarker()
        mt.id = tool_id
        mt.pose.pose = self.tool_pose
        m_all.markers.append(mt)

        return m_all

    def update(self):
        m = self.get_markers()
        self.marker_publisher.publish(m)
        time.sleep(0.1)


if __name__ == "__main__":

    rospy.init_node('pr2_pbd_simworld', anonymous=True)

    sw = SimWorld()
    while(not rospy.is_shutdown()):
        sw.update()