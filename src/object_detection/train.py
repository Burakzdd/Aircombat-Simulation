from ultralytics import YOLO
import pandas
# Load a model
model = YOLO("/home/burakzdd/Desktop/air_combat_simulation2/src/object_detection/uav/weights/best.pt")  # load a pretrained model (recommended for training)

result = model("/home/burakzdd/Desktop/air_combat_simulation2/src/object_detection/data/valid/images/uav_1.jpg")  # predict on an image

# boxes = result.boxes  # Boxes object for bounding box outputs
# probs = result.probs  # Probs object for classification outputs
# obb = result.obb  # Oriented boxes object for OBB outputs
result.show()  # display to screen
result.save(filename='result.jpg')  # save to disk