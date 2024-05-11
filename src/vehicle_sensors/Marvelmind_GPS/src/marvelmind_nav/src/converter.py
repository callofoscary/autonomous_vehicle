#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from marvelmind_nav.msg import hedge_pos_ang


class CombinerNode():
    def __init__(self):
        rospy.init_node("marvelmind_message")

        #Subscribers
        self.subscriber_one = rospy.Subscriber("/pose",Pose,self.callback_yaw)
        self.subscriber_two = rospy.Subscriber("/hedge_pos_ang",hedge_pos_ang,self.callback_position)

        #Publishers
        self.publisher = rospy.Publisher("/marvelmind_message",PoseStamped,queue_size=10)

        #Data Storage
        self.data_yaw = None
        self.data_position=None

    def callback_yaw(self,data):
        self.data_yaw = data.orientation
        self.try_publish()
    
    
    def callback_position(self,data):
        self.data_position= (data.x_m,data.y_m,data.z_m)
        self.try_publish()
    
    def try_publish(self):
        if self.data_position is not None and self.data_yaw is not None:
            combined_pose = PoseStamped()
            combined_pose.header.stamp = rospy.Time.now()
            combined_pose.header.frame_id = "map"

            combined_pose.pose.position.x = self.data_position[0]
            combined_pose.pose.position.y = self.data_position[1]
            combined_pose.pose.position.z = self.data_position[2]
            combined_pose.pose.orientation = self.data_yaw

            self.publisher.publish(combined_pose)

if __name__ == "__main__":
    node = CombinerNode()
    rospy.spin()
