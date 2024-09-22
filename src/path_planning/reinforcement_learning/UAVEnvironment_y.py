import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, TwistStamped,Twist
from std_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
import random

class UAVEnvironment_y:
    def __init__(self):
        # self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(20.0)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        
        self.state_size = 1 # Durum uzayının boyutu (x, y, z,)
        self.action_size = 1  # Eylem uzayının boyutu (x, y, z hızları)
        self.threshold = 1  # Hedefe olan mesafe eşiği
        self.target_position = np.array([0, 30, 5])  # Hedef konumu
        self.max_steps = 300 # Maksimum adım sayısı
        self.current_step = 0  # Başlangıç adımı
        self.current_position = Point()
        self.k_p = 0.01
    def set_target_position(self):
        self.target_position = random.randint(-50,50)
        
    def get_reward(self, current_position):
        rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        # distance = np.linalg.norm(self.target_position - current_position)
        if abs(self.target_position[1]-self.current_position.y) <= self.threshold:
            print("Done, Mesafe: ",abs(self.target_position[1]-self.current_position.y))
            return 100  # Hedefe 1 birim veya daha yakın olduğunda +100 puan
        else:
            return abs(self.target_position[1]-self.current_position.y)* -self.k_p

            # return -distance*self.k_p  # Hedefe olan mesafenin negatifini alarak ödül

    def step(self,action):
        self.current_step += 1
        reward = self.get_reward(self.get_state())
        # print(reward)
        if reward == 100:
            done = True
        else:
            done = False
            if self.current_step > self.max_steps:
                reward = -100
                self.current_step = 0
        self.set_velocity(action)
        
        next_state = self.get_state()
        return next_state, reward, done
    
    def local_pose_cb(self, msg):
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z
        
    def get_state(self):
        rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        return np.array([self.target_position[1]-self.current_position.y])#,self.current_position.y,self.current_position.z])
    
    def set_velocity(self,action):
        actions = np.arange(-1.0, 1.1, 0.1)
        velocity = Twist()
        velocity.linear.x = 0 # x hızı
        velocity.linear.y = actions[action[0]]#action[0][1]  # y hızı
        velocity.linear.z = 0 #action[0][2]  # z hızı
        # Aksiyon mesajını yayınla
        self.pub_vel.publish(velocity)
        rospy.sleep(0.1)
        
    def takeoff(self):
        rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        while self.current_position.z < 5:
            rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
            msg = Twist()
            # msg.header.stamp = rospy.Time.now()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 1
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)  # Döngüyü her iterasyonda bir süre beklet
        msg = Twist()
        msg.linear.z = 0
        self.pub_vel.publish(msg)    

    
        
    def clear(self):
        state_msg = ModelState()
        state_msg.model_name = 'quadrotor'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 10.0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1.0
        
        target_twist = Twist()
        target_twist.linear.x = 0.0
        target_twist.linear.y = 0.0
        target_twist.linear.z = 0.0
        target_twist.angular.x = 0.0
        target_twist.angular.y = 0.0
        target_twist.angular.z = 0.0
        state_msg.twist = target_twist
        self.set_state(state_msg)

        # self.takeoff()
        self.rate.sleep()
        # state_msg.pose.position.x = 0
        # state_msg.pose.position.y = 0
        # state_msg.pose.position.z = 10.0
        # state_msg.pose.orientation.x = 0
        # state_msg.pose.orientation.y = 0
        # state_msg.pose.orientation.z = 0
        # state_msg.pose.orientation.w = 0.0

        # self.set_state(state_msg)
        # rospy.loginfo("Model state set successfully.")
        
    
    def callback(self, msg_odom):
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
        # rot_q = msg_odom.pose.pose.orientation
        # (roll, pitch, self.yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])