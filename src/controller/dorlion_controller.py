import rospy
import random
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
import torch
from PyQt5.QtCore import pyqtSignal, QObject
from tkinter import image_names
import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
import torchvision
import torch.nn as nn
import cv2
from PIL import Image as pil
import os
import time
class DorlionController(QObject):
    image_signal = pyqtSignal(np.ndarray)

    def __init__(self, takeoff_altitude=10):
        super().__init__()
        self.i = 0
        self.rate = rospy.Rate(20.0)
        self.pub_vel = rospy.Publisher("/dorlion/cmd_vel", Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("/dorlion/ground_truth/state", Odometry, self.pose_callback)
        self.takeoff_altitude = takeoff_altitude
        self.current_position = Point()
        self.speed_x = 0.1
        self.roll,self.pitch,self.yaw = 0.0,0.0,0.0
        self.dorlionPosition = Point()
        self.dangerUav = Point()
        self.enemys = []
        self.detection_status = False
        
        self.bridge = CvBridge()
        rospy.Subscriber("/dorlion/camera/rgb/image_raw",Image,self.cameraCallback)
        self.model = YOLO("/home/burakzdd/catkin_ws/src/air_combat_simulation/src/object_detection/uav_gpu_model3/weights/best.pt")
        self.model.to(torch.device('cuda'))
        self.image = []
        
        self.center_x = 0
        self.center_y = 0
        self.w = 0
        self.h = 0
        self.tracking_started = False
        self.countdown = 10
        
    def cameraCallback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        self.image = cv2.resize(self.image, (600,400), interpolation= cv2.INTER_LINEAR)
        self.h,self.w,_ = self.image.shape
        # if self.detection_status == False:
        #     self.image_signal.emit(self.image)

    def pose_callback(self, msg_odom):
        self.current_position.x = msg_odom.pose.pose.position.x
        self.current_position.y = msg_odom.pose.pose.position.y
        self.current_position.z = msg_odom.pose.pose.position.z
        rot_q = msg_odom.pose.pose.orientation
        (self.roll,self.pitch,self.yaw) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        
    def setDangerUav(self,pos):
        self.dangerUav = pos
        
    def getDangerUav(self):
        return self.dangerUav
        
    def getPosition(self):
        return self.current_position

    def takeoff(self):
        
        while self.current_position.z < self.takeoff_altitude:
            msg = Twist()
            # msg.linear.x = 1
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
        print("Dorlion - take off successfully")

    def set_randomly_speed(self):
        msg = Twist()
        msg.linear.x = np.random.uniform(0, 1)
        msg.linear.y = np.random.uniform(0, 1)
        msg.linear.z = 0
        self.pub_vel.publish(msg)
    
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
        
    def enemy_randomly_speed(self):
        (self.enemys[0]).set_randomly_speed(10)
        (self.enemys[1]).set_randomly_speed(10)
        (self.enemys[2]).set_randomly_speed(10)
        (self.enemys[3]).set_randomly_speed(10)
        
    def move_path(self,points,log_signal,info_str_1):
        self.enemy_randomly_speed()
        goal = Point()
        # current = Point()
        speed = Twist()
        points = points[1:]
        for point in points:
            self.enemy_randomly_speed()
            rospy.Subscriber("/dorlion/ground_truth/state", Odometry, self.pose_callback)
            # print("Current Position: [{} {} {}]".format(self.current_position.x,self.current_position.y,self.current_position.z))
            log_signal.emit(info_str_1 + f"Hedef nokta:{point}\nUçak hedefe yönelmektedir!")
            # print("Hedef nokta",point)

            # print()
            
            angle_to_goal = math.atan2(point[1]-self.current_position.y, point[0]-self.current_position.x)
            
            # distance_to_goal_xy = math.sqrt((point[0] - self.current_position.x)**2 + (point[1] - self.current_position.y)**2)
            # angle_to_goal_pitch = math.atan2(point[2] - self.current_position.z, distance_to_goal_xy)
            # angle_difference_pitch = angle_to_goal_pitch - self.pitch
            
            while abs(angle_to_goal - self.yaw) > 0.3:
                self.enemy_randomly_speed()
                
                speed.linear.x = 0.0
                speed.angular.z = 2 * (angle_to_goal - self.yaw)
                # speed.angular.y =  0.3* (angle_to_goal_pitch - self.pitch)
                self.pub_vel.publish(speed)
            lenght = math.sqrt((point[0]-self.current_position.x)**2+(point[1]-self.current_position.y)**2+(point[2]-self.current_position.z)**2)
            speed.linear.x = 0.5
            speed.angular.z = 0.0
            speed.angular.y = 0.0
            log_signal.emit(info_str_1 + f"Hedef nokta: {point}\nUçak hedefe gitmektedir!")

            while abs(lenght) > 0.7:
                self.enemy_randomly_speed()
                # print("z hızı:",min(((point[2] - self.current_position.z )* 0.5),0.2))
                speed.linear.z = min(((point[2] - self.current_position.z )* 0.5),0.2)
                speed.linear.x = min((lenght * 0.5),0.2)
                self.pub_vel.publish(speed)
                lenght = math.sqrt((point[0]-self.current_position.x)**2+(point[1]-self.current_position.y)**2+(point[2]-self.current_position.z)**2)
            speed.linear.x = 0.0
            speed.linear.y = 0.0
            speed.linear.z = 0.0
            self.pub_vel.publish(speed)

        angle_to_uav = math.atan2(self.dangerUav.getPosition().y-self.current_position.y, self.dangerUav.getPosition().x-self.current_position.x)
        while abs(angle_to_uav - self.yaw) > 0.4:
            self.enemy_randomly_speed()
            
            speed.linear.x = 0.0
            speed.angular.z = (0.2 * (angle_to_uav - self.yaw))
            speed.linear.z = ((self.dangerUav.getPosition().z - self.current_position.z )* 0.15)
            self.pub_vel.publish(speed)
        speed.angular.z = 0.0
        speed.linear.z = 0.0 
        speed.linear.x = 0.0
        self.pub_vel.publish(speed)
        enemy_uav_angle = abs(self.yaw - self.getDangerUav().getYaw())
        print("Flight succesfully")
        
    def scan_uav(self):
        msg = Twist()
        msg.angular.z = 0.1
        self.pub_vel.publish(msg)
  
      
    def detection(self):
        self.bbox_percentage = 70
        self.detection_status = True
        x1,x2,y1,y2 = 0,0,0,0
        # rospy.Subscriber("/dorlion/camera/rgb/image_raw",Image,self.cameraCallback)
        results = self.model.predict(self.image)
        result = results[0]
        highest_prob = 0
        output = []
        best_box = None
        best_class_id = None
        for box in result.boxes:
            x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
            class_id = box.cls[0].item()
            prob = round(box.conf[0].item(), 2)

            if prob > highest_prob:
                highest_prob = prob
                best_box = [x1, y1, x2, y2]
                best_class_id = class_id

        if best_box is not None:
            output.append(best_box + [result.names[best_class_id], highest_prob])
            self.center_x = best_box[0] + (best_box[2] - best_box[0]) / 2
            self.center_y = best_box[1] + (best_box[3] - best_box[1]) / 2
            # print("Center Point:", self.center_x, self.center_y, sep=" ")
            bbox_area = abs(best_box[2] - best_box[0]) * abs(best_box[3] - best_box[1])
            frame_area = self.h * self.w
            self.bbox_percentage = (bbox_area / frame_area) * 100
            # print(f"Bounding Box Percentage of Frame: {self.bbox_percentage:.2f}%")
 
        if not self.tracking_started:
            self.countdown = 10
        elif self.countdown == 0:
            pass
        elif abs(self.time1 - time.time())> 1:
            self.countdown -= 1
            self.time1 = time.time()
                  
        img = self.draw_boxes(self.image,output)
        self.display_countdown(img)

        if output == []:
            self.scan_uav()
        
        # img = cv2.resize(img, (600, 400),interpolation = cv2.INTER_LINEAR)
        self.image_signal.emit(img)
        # cv2.imshow("UAV DETECTION",img)
        # cv2.waitKey(1)
    
    def tracking(self):
        if self.center_x != 0:
            if self.i == 0:
                self.time1 = time.time()
                self.tracking_started = True
                self.i = -1
            sapma = self.center_x - self.w/2
            self.speed = Twist()
            print(self.bbox_percentage)
            
            if self.bbox_percentage >= 13:
                self.speed.linear.x = 0.0
            
            elif self.bbox_percentage >= 9:
                self.speed.linear.x = 0.1
            
            else:
                self.speed.linear.x =(0.5 - (self.bbox_percentage)/10)
            self.speed.linear.z = (self.center_y - (self.h/2))/-1000
            self.speed.angular.z = -sapma/1000
            self.pub_vel.publish(self.speed)

        
    def draw_boxes(self,img, output):

        for box in output:
            x1, y1, x2, y2, class_name, prob = box
            # color = (np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(img, (int(self.center_x), int(self.center_y)), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(img, (self.w//2, self.h//2), radius=3, color=(255, 0, 0), thickness=-1)

            label = f"{class_name} ({prob})"
            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return img
    
    def setVelocity(self,x,y,z,r,p,yaw):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = r
        msg.angular.y = p
        msg.angular.z = yaw
        self.pub_vel.publish(msg)
        
    def display_countdown(self,image):
        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.countdown == 0:
            cv2.putText(image, "Flight succesfully", (30, 80), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(image, str(self.countdown), (30, 80), font, 2, (0, 0, 255), 3, cv2.LINE_AA)
        # self.image_signal.emit(self.image)
        
        
    def set_randomly_speed(self,over=5):
        msg = Twist()

        msg.linear.x = random.choice([0.05,0.01,0.02,0.5,0.3,0.2,0.4,0.45,0.6,0.7,0.8,0.9,0.1,0.15,0.16])
        msg.linear.y = random.uniform(-1, 1)/over
        msg.linear.z = random.uniform(-1, 1)/over
        msg.angular.x = random.uniform(-1, 1)/over
        msg.angular.y = random.uniform(-1, 1)/over
        msg.angular.z = random.uniform(-1, 1)/over
        self.pub_vel.publish(msg)

