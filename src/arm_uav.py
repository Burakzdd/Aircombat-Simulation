#!/usr/bin/env python3


from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time
import rospy

class Arm:
    def __init__(self,name,takeoff_amplitude=1):
        self.name = name
        self.pub_vel = rospy.Publisher(f'/{name}/cmd_vel', Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber(f'/{name}/ground_truth/state', Odometry, self.pose_callback)
        self.current_position = Point()
        self.takeoff_amplitude = takeoff_amplitude
    
    def run(self):
       
        while self.current_position.z < self.takeoff_amplitude:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 1
            self.pub_vel.publish(msg)
            rospy.sleep(0.1) 
        msg = Twist()
        msg.linear.z = 0
        self.pub_vel.publish(msg)
        print(f"{self.name} - take off successfully")
    
    def pose_callback(self, msg_odom):
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
    
    def stop(self):
    
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0    
        self.pub_vel.publish(msg)

def main():
    rospy.init_node('takeOff_node'.format(1), anonymous=True)

    name_list = ["dorlion","uav1","uav2","uav3","uav4"]
    
    for name in name_list:
        uav = Arm(name)
        
        uav.run()
        
main()