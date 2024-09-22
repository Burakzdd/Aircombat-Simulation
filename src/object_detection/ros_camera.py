import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
class Camera:
    def __init__(self):
        print("Camera node ")
        rospy.init_node("node_camera")
        self.bridge = CvBridge()
        rospy.Subscriber("/dorlion/camera/rgb/image_raw",Image,self.cameraCallback)
        self.model = YOLO("/home/burakzdd/catkin_ws/src/air_combat_simulation/src/object_detection/uav/weights/best.pt")
        print("load model")
        self.image = []
   
        
    def cameraCallback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        
    def detection(self):
        rospy.Subscriber("/dorlion/camera/rgb/image_raw",Image,self.cameraCallback)
        print("enter detection")
        print("image:",self.image)
        results = self.model.predict(self.image)
        print(results)
        result = results[0]
        output = []
        for box in result.boxes:
            x1, y1, x2, y2 = [
                round(x) for x in box.xyxy[0].tolist()
            ]
            class_id = box.cls[0].item()
            prob = round(box.conf[0].item(), 2)
            output.append([
                x1, y1, x2, y2, result.names[class_id], prob
            ])
        img = self.draw_boxes(self.image,output)
        
        cv2.imshow("frame",img)
        cv2.waitKey(1)
        
    def draw_boxes(self,img, output):
        # Resmi yükle
        
        # Her bir çıktı için
        for box in output:
            x1, y1, x2, y2, class_name, prob = box
            # Rastgele bir renk oluştur
            color = (np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256))
            # Bounding box çiz
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            # Etiket yaz
            label = f"{class_name} ({prob})"
            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return img
def main():
    
    cam = Camera()
    while True:
        print("in while")
        cam.detection()

main()