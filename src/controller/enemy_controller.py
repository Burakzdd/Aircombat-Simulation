import rospy
import random
import numpy as np
import skfuzzy as fuzz
import skfuzzy.membership as mf
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from findDangerPlane.find_position import calculate_Position
from gazebo_msgs.srv import DeleteModel

class EnemyController():
    
    def __init__(self, ns, takeoff_altitude=13):
        self.ns = ns

        self.rate = rospy.Rate(20.0)
        self.pub_vel = rospy.Publisher("/{}/cmd_vel".format(self.ns), Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("/{}/ground_truth/state".format(self.ns), Odometry, self.pose_callback)
        self.sub_vel =rospy.Subscriber("/{}/cmd_vel".format(self.ns), Twist, self.vel_callback) # "/uav1/cmd_vel" konumundan Twist mesajlarını al ve callback fonksiyonunu çağır
        self.takeoff_altitude = takeoff_altitude
        self.odom = None
        self.current_position = Point()
        self.speed_x = 0.1
        
        self.dorlionPosition = Point()
        self.yaw = 0.0

    def pose_callback(self, msg_odom):
        self.odom = msg_odom
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
        rot_q = msg_odom.pose.pose.orientation
        (self.roll,self.pitch,self.yaw) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        
    def getYaw(self):
        return self.yaw
    
    def getPosition(self):
        return self.current_position

    def vel_callback(self, msg_twist):
        # self.speed_x = 0.0
        self.speed_x = msg_twist.linear.x
    
    def getVel(self):
        rospy.Subscriber("/{}/cmd_vel".format(self.ns), Twist, self.vel_callback)
        if self.speed_x == 0.0:
            self.speed_x = 0.1
        return self.speed_x

    
    def takeoff(self):
        rospy.Subscriber("/{}/ground_truth/state".format(self.ns), Odometry, self.pose_callback)
        while self.current_position.z < self.takeoff_altitude:
            
            rospy.Subscriber("/{}/ground_truth/state".format(self.ns), Odometry, self.pose_callback)
            msg = Twist()
            msg.linear.x = 0.5
            msg.linear.y = 0
            msg.linear.z = 1
            msg.angular.z = 0
            self.pub_vel.publish(msg)
            rospy.sleep(0.1) 
        msg = Twist()
        msg.linear.z = 0.0
        msg.linear.x = 0.0
        self.pub_vel.publish(msg)
        print("{} - take off successfully".format(self.ns))
        
    def calculate_distance(self):
        return math.sqrt((self.current_position.x-self.dorlionPosition.x)**2
                        +(self.current_position.y-self.dorlionPosition.y)**2
                        +(self.current_position.z-self.dorlionPosition.z)**2)
        
    def get_features(self):
        rospy.Subscriber("/{}/ground_truth/state".format(self.ns), Odometry, self.pose_callback)
        rospy.Subscriber("/{}/cmd_vel".format(self.ns), Twist, self.vel_callback)
        return self.ns,self.calculate_distance(), self.getVel(), calculate_Position(self.dorlionPosition,self.current_position,self.odom)
        
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
    
    def generate_random(self):
        num = 0
        while num == 0:
            num = random.uniform(0, 1)
        return num
    
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
    
    def setDorlion(self,dorlion_position):
        self.dorlionPosition = dorlion_position
        
        
    def setVelocity(self,x,y,z,r,p,yaw):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = r
        msg.angular.y = p
        msg.angular.z = yaw
        self.pub_vel.publish(msg)
        
    
    def deleteEnemy(self):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            response = delete_model_service(self.ns)
            if response.success:
                rospy.loginfo(f"Model '{self.ns}' başarıyla silindi.")
            else:
                rospy.logwarn(f"Model '{self.ns}' silinemedi: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Servis çağrısı başarısız oldu: {e}")     