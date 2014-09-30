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
        self.tool_pose = Pose(Point(0.5, 0.0, 1.2), Quaternion(0, 1, 0, 1))

        self.table_w = 0.4
        self.table_h = 0.3
        self.table_z = 0.65

        self.update_table_markers()

    def update_table_markers(self):
        self.table_poses = {
            1: Pose(Point(0.4, self.table_w/2.0, self.table_z), Quaternion(0, 0, 1, 1)),
            2: Pose(Point(0.4, -self.table_w/2.0, self.table_z), Quaternion(0, 0, 1, 1)),
            3: Pose(Point(0.4 + self.table_h, self.table_w/2.0, self.table_z), Quaternion(0, 0, 1, 1)),
            4: Pose(Point(0.4 + self.table_h, -self.table_w/2.0, self.table_z), Quaternion(0, 0, 1, 1))
        }

    def get_markers(self):
        m_all = AlvarMarkers()

        tool_id = 99
        if rospy.has_param('cleaning_tool_id'):
            tool_id = rospy.get_param('cleaning_tool_id')
            
        if rospy.has_param('table_w'):
            table_w = rospy.get_param('table_w')
            if self.table_w != table_w:
                rospy.loginfo('Table width changed.')
                self.table_w = table_w
        if rospy.has_param('table_h'):
            table_h = rospy.get_param('table_h')
            if self.table_h != table_h:
                rospy.loginfo('Table height changed.')
                self.table_h = table_h

        self.update_table_markers()

        for k in self.table_poses.keys():
            m = AlvarMarker()
            m.id = k
            m.pose.pose = self.table_poses[k]
            m_all.markers.append(m)

        mt = AlvarMarker()
        mt.id = tool_id
        mt.pose.pose = self.tool_pose
        #m_all.markers.append(mt)

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