#!/usr/bin/env python

import rospy
import rosbag
import rospkg
import time
import visualization_msgs.msg
import numpy as np
from std_msgs.msg import Int64

class SimpPath(object):
    def __init__(self):
        self.sub_filtered = rospy.Subscriber('/filtered_path', Int64, self.callback_filtered, queue_size=1)
        self.pub_points = rospy.Publisher('/simplified_path', visualization_msgs.msg.MarkerArray, queue_size=1)
        self.pub_points_filtered = rospy.Publisher('/simplified_path_filtered', visualization_msgs.msg.MarkerArray, queue_size=1)
        self.tim_10hz = rospy.Timer(rospy.Duration(0.1), self.timer_10hz)

        # -----------------------

        self.path = []
        self.simppath = []

        # -----------------------

        self.read_bag()
        rospy.sleep(2)

    # --------------------------------------------------
        
    def callback_filtered(self, msg):
        self.number_of_points = msg.data
        self.simppath = self.filter_path()
        self.publish_point_filtered()

        print("Number of points in the path: ", len(self.path), " | Number of points in the simplified path: ", len(self.simppath))

    # --------------------------------------------------
        
    def timer_10hz(self, event):
        self.publish_point_raw()

    # --------------------------------------------------
            
    def filter_path(self):
        if len(self.path) <= self.number_of_points:
            print("Number of points in the path is already less than or equal to the desired number of points.")
            return self.path
        
        # -----------------------

        simplified_path = [[self.path[0][0] + 1, self.path[0][1]]]

        if(self.number_of_points <= 2):
            simplified_path.append([self.path[-1][0] + 1, self.path[-1][1]]) 

            return simplified_path
        
        # -----------------------

        segment_length = len(self.path) / float(self.number_of_points - 1)

        for i in range(self.number_of_points - 1):
            next_point = int(round((i + 1) * segment_length))
            
            if next_point < len(self.path) - 1:
                simplified_path.append([self.path[next_point][0] + 1, self.path[next_point][1]])
            else:
                simplified_path.append([self.path[-1][0] + 1, self.path[-1][1]])

        return simplified_path

    # --------------------------------------------------

    def read_bag(self):
        rospack = rospkg.RosPack()
        bag = rosbag.Bag(rospack.get_path('movement_controller') + '/bags/path_test.bag')
        for topic, msg, t in bag.read_messages():
            path_now = [msg.pose.position.x, msg.pose.position.y]
            self.path.append(path_now)
        bag.close()
        self.is_read_bag = True

    # --------------------------------------------------
    
    def publish_point_filtered(self):
        marker_array = visualization_msgs.msg.MarkerArray()
        for i in range(0, len(self.simppath)):
            marker = visualization_msgs.msg.Marker()
            marker.type = 3
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "simplified_path_filtered"
            marker.id = i
            marker.action = visualization_msgs.msg.Marker.DELETEALL
            marker.pose.position.x = self.simppath[i][0]
            marker.pose.position.y = self.simppath[i][1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.pub_points_filtered.publish(marker_array)

        # --------------------------------------------------

        for i in range(0, len(self.simppath)):
            marker = visualization_msgs.msg.Marker()
            marker.type = 3
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "simplified_path_filtered"
            marker.id = i
            marker.action = visualization_msgs.msg.Marker.ADD
            marker.pose.position.x = self.simppath[i][0]
            marker.pose.position.y = self.simppath[i][1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.pub_points_filtered.publish(marker_array)

    # --------------------------------------------------
            
    def publish_point_raw(self):
        marker_array = visualization_msgs.msg.MarkerArray()
        for i in range(0, len(self.path)):
            marker = visualization_msgs.msg.Marker()
            marker.type = 3
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "simplified_path"
            marker.id = i
            marker.action = visualization_msgs.msg.Marker.ADD
            marker.pose.position.x = self.path[i][0]
            marker.pose.position.y = self.path[i][1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.pub_points.publish(marker_array)

# --------------------------------------------------
# ==================================================

if __name__ == '__main__':
    rospy.init_node('simplify_path')
    simppath = SimpPath()
    rospy.spin()