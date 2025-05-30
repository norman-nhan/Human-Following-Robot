import torch
import numpy as np
import cv2
from ultralytics import YOLO

model = YOLO("./train2N/weights/best.pt")

def predict(img):
    # Get original image size
    h_orig, w_orig = img.shape[:2]

    # Run YOLO prediction (no need to resize manually)
    results = model(img) 
    boxes = results[0].boxes.data  # shape: (N, 6)

    if boxes is None or boxes.shape[0] == 0:
        print("No detections")
        return None, None

    # Filter for legs (class == 0)
    legs = boxes[boxes[:, 5] == 0]
    if legs.shape[0] == 0:
        print("No legs detected")
        return None, None

    # Compute area of each leg
    areas = (legs[:, 2] - legs[:, 0]) * (legs[:, 3] - legs[:, 1])
    max_area_idx = torch.argmax(areas)
    biggest_leg = legs[max_area_idx]

    # Center in 640x640
    x_center = (biggest_leg[0] + biggest_leg[2]) / 2
    y_center = (biggest_leg[1] + biggest_leg[3]) / 2

    # Scale center to original image
    x_center_orig = x_center.item() * w_orig / 640
    y_center_orig = y_center.item() * h_orig / 640
    center = (int(x_center_orig), int(y_center_orig))

    # Draw circle on the image
    img_with_center = img.copy()
    cv2.circle(img_with_center, center, radius=6, color=(0, 0, 255), thickness=-1)
    cv2.putText(img_with_center, "Center", (center[0] + 10, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Show image
    cv2.imshow("Image with center", img_with_center)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return biggest_leg.tolist(), center
