import numpy as np
import rospy
from geometry_msgs.msg import Point,Twist
from std_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import random
import subprocess

class UAVEnvironment:
    def __init__(self):
        
        self.rate = rospy.Rate(20.0)
        
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.state_size = 1 
        self.action_size = 1  
        self.threshold = 1 
        
        self.target_position = 0
        self.max_steps = 10000
        self.current_step = 0  
        self.current_position = Point()
        self.k_p = 0.1
        self.x = 0

    def set_target_position(self):
        self.target_position = 10#random.randint(-20 ,50)

    def get_reward(self):
        rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        if abs(self.target_position-self.current_position.x) <= self.threshold:
            print("Done, Mesafe: ",abs(self.target_position-self.current_position.x))
            return 100
        else:
            return abs(self.target_position-self.current_position.x) * -self.k_p

    def step(self,action):
        self.current_step += 1
        reward = self.get_reward()
        if reward == 100:
            done = True
            
        else:
            done = False
            if self.current_step > self.max_steps:
                reward = -100
                
        self.set_velocity(action)
        next_state = self.get_state()
        
        return next_state, reward, done
    
    def local_pose_cb(self, msg):
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z
        
    def get_state(self):
        return np.array([(self.target_position-self.current_position.x)*0.1])
    
    def set_velocity(self,action):
        actions = np.arange(-1.0, 1.1, 0.1)
        velocity = Twist()
        
        velocity.linear.x = actions[action[0]] 
        velocity.linear.y = 0
        velocity.linear.z = 0

        self.pub_vel.publish(velocity)
        
    def takeoff(self):
        while self.current_position.z < 5:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 1
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)  
            
        msg = Twist()
        msg.linear.z = 0
        self.pub_vel.publish(msg)    
    
    def clear(self):
        self.current_step = 0

        # rospy.wait_for_service("/gazebo/reset_world")
        # try:
        #     subprocess.call(["rosservice", "call", "/gazebo/reset_world"])
        # except rospy.ServiceException as e:
        #     print("/gazebo/reset_world service call failed")

        state_msg = ModelState()
        state_msg.model_name = 'uav'
        self.x = 0
        state_msg.pose.position.x = self.x
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.5
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
        
        try:
            response = self.set_model_state(state_msg)
            if not response.success:
                print("Setting model state failed!")
        except rospy.ServiceException as e:
            print("Service call failed:", e)
        
        rospy.sleep(1)

    
    def callback(self, msg_odom):
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
        # rot_q = msg_odom.pose.pose.orientation
        # (roll, pitch, self.yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])