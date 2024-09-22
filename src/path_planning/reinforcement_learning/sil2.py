#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class DroneController:
    def __init__(self):
        # ROS node başlatma
        rospy.init_node('drone_control', anonymous=True)
        self.current_z = 0.0
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odometry_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.is_rising = True

    def odometry_callback(self, data):
        # Odometry verisinden z konumunu güncelle
        self.current_z = data.pose.pose.position.z

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            print(self.current_z)
            if self.current_z < 5.0 and self.is_rising:
                # Drone'u yükselt
                twist = Twist()
                twist.linear.z = 1.0
                self.cmd_vel_pub.publish(twist)
            elif self.current_z >= 5.0:
                self.is_rising = False
                # Z konumu 5 metreye ulaştığında ileri hareketi başlat
                twist = Twist()
                twist.linear.x = 0.4
                self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    drone_controller = DroneController()
    drone_controller.run()
