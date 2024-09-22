from ultralytics import YOLO
import cv2
import numpy as np
import torch

# Path to the image and model weights
path = "data/valid/images/uav_1.jpg"
model_path = "/home/burakzdd/catkin_ws/src/air_combat_simulation/src/object_detection/uav_2_100/weights/best.pt"

# Load the image using OpenCV
img = cv2.imread(path)

# Initialize the model and set the device
model = YOLO(model_path)
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model.to(device)

def draw_boxes(image_path, output):
    # Load the image
    image = cv2.imread(image_path)
    
    # Draw each bounding box on the image
    for box in output:
        x1, y1, x2, y2, class_name, prob = box
        # Generate a random color for the bounding box
        color = (np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256))
        # Draw the bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        # Write the label above the bounding box
        label = f"{class_name} ({prob})"
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    return image

def inference(image):
    global model
    image = torch.from_numpy(image).to(device).float()
    results = model.predict(image)
    result = results[0]
    output = []
    for box in result.boxes:
        x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
        class_id = box.cls[0].item()
        prob = round(box.conf[0].item(), 2)
        output.append([x1, y1, x2, y2, result.names[class_id], prob])
    return output

# Perform inference on the image
output = inference(img)

# Draw the bounding boxes on the image
out_img = draw_boxes(path, output)

# Display the result
cv2.imshow("Result", out_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
