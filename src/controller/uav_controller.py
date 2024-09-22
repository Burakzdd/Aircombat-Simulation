import rospy
import random
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class UAVController:
    def __init__(self, ns, takeoff_altitude=10):
        self.ns = ns
        self.takeoff_altitude = takeoff_altitude

        self.rate = rospy.Rate(20.0)
        self.pub_vel = rospy.Publisher("/{}/cmd_vel".format(self.ns), Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("/{}/ground_truth/state".format(self.ns), Odometry, self.pose_callback)
        self.current_position = Point()
        self.speed_x = 0.1
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.dorlionPosition = Point()
        
    def pose_callback(self, msg_odom):
        self.odom = msg_odom
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
        rot_q = msg_odom.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
    def getYaw(self):
        return self.yaw
    
    def getPosition(self):
        return self.current_position

    def set_randomly_speed(self,over=5):
        msg = Twist()
        if over == 10:
            msg.linear.x = 0.1
        else:
            msg.linear.x = random.choice([0.05,0.01,0.02,0.5,0.3,0.2,0.4,0.45,0.6,0.7,0.8,0.9,0.1,0.15,0.16])
        msg.linear.y = random.uniform(-1, 1)/over
        msg.linear.z = random.uniform(-1, 1)/over
        msg.angular.x = random.uniform(-1, 1)/over
        msg.angular.y = random.uniform(-1, 1)/over
        msg.angular.z = random.uniform(-1, 1)/over
        self.pub_vel.publish(msg)
    
    
    def takeoff(self):
        while self.current_position.z < self.takeoff_altitude:
            msg = Twist()
            msg.linear.y = 0
            msg.linear.z = 1
            msg.angular.x = 0.5
            self.pub_vel.publish(msg)
            rospy.sleep(0.1) 
        msg = Twist()
        msg.linear.x = 0
        msg.linear.z = 0
        msg.angular.z = 0
        msg.angular.x = 0
        self.pub_vel.publish(msg)
        print("{} - take off successfully".format(self.ns))
    
    def stop_uav(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.pub_vel.publish(msg)

    def set_position_randomly(self):
        sp = PoseStamped()
        sp.pose.position.x = random.uniform(50, 150)
        sp.pose.position.y = random.uniform(50, 150)
        sp.pose.position.z = self.takeoff_altitude + 10

        self.sp_pub.publish(sp)
        self.rate.sleep()
    
    def setVelocity(self,x,y,z,r,p,yaw):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = r
        msg.angular.y = p
        msg.angular.z = yaw
        self.pub_vel.publish(msg)

    def calculate_distance(self, other_position):
        return math.sqrt((self.current_position.x - other_position.x)**2 +
                         (self.current_position.y - other_position.y)**2 +
                         (self.current_position.z - other_position.z)**2)

