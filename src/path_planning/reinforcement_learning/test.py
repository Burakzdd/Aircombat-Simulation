import torch
from DQNModel import DQN
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
import numpy as np
import math

class evaluate():
    def __init__(self,model_path):
        self.model = DQN()
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        self.current_position = Point()
        self.hedef = 100
        
    def distance(self):
        return abs(self.current_position.x - self.hedef)
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        self.pub_vel.publish(msg)

    def findAction(self,state):

        state = torch.FloatTensor(state).unsqueeze(0)
        # actions = self.model(state)
        actions = self.model(state).max(1).indices.view(-1, 1)
        # actions = actions[0].max(1).indices.view(-1, 1)#,actions[1].max(1).indices.view(-1, 1),actions[2].max(1).indices.view(-1, 1)
        return actions
        
    def get_state(self):
        rospy.Subscriber("/ground_truth/state", Odometry, self.callback)
        print(self.hedef-self.current_position.x)
        return np.array([self.hedef-self.current_position.x])
        # return np.array([self.current_position.x,self.current_position.y,self.current_position.z])

    def callback(self, msg_odom):
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
        
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

    def set_velocity(self,action):
        actions = np.arange(-1.0, 1.1, 0.1)
        print(action)
        print(actions[action[0]])
        velocity = Twist()
        
        velocity.linear.y = 0 # x hızı
        velocity.linear.x = actions[action[0]]#actions[action[1]]#action[0][1]  # y hızı
        velocity.linear.z = 0#actions[action[2]]#action[0][2]  # z hızı
        # Aksiyon mesajını yayınla
        self.pub_vel.publish(velocity)
        rospy.sleep(0.1)

def main():
    plane = evaluate("/home/burakzdd/catkin_ws/src/deneme/scripts_one_axis/model/model.pth")
    plane.takeoff()
    while plane.distance() > 1:
        action = plane.findAction(plane.get_state())
        plane.set_velocity(action)
    plane.stop()
    print("hedefe ulaşıldı")
    
if __name__ == "__main__":
    rospy.init_node('evaluate', anonymous=True)
    main()