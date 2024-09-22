import cv2
import os

def video_to_frames(video_path, output_path):
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Video dosyası yüklenemedi.")
        return
    
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    frame_count = 0
    count= 0
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        if frame_count % 15 == 0:
            count +=1
            frame_name = f"uav_{count}.jpg"
            frame_path = os.path.join(output_path, frame_name)
            cv2.imwrite(frame_path, frame)
            print(f"{frame_name} kaydedildi.")
        
        frame_count += 1
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    video_path = "/home/burakzdd/catkin_ws/src/air_combat_simulation/object_detection/data/uav.mp4"
    output_path = "/home/burakzdd/catkin_ws/src/air_combat_simulation/object_detection/data/frames"
    
    video_to_frames(video_path, output_path)
