Code: State 3 (Ali model, with voice, red band)
#Setup Instructions
pip install pyttsx3

import cv2
import numpy as np
import time
import pyttsx3

# === Setup for text-to-speech ===
engine = pyttsx3.init()
engine.setProperty('rate', 150)  # speed of speech

def scream_behavior():
    print("😱 AHHHHH!! I LOST THE NURSE!! SEARCHING...")
    engine.say("AHHHHH!! I lost the nurse! Searching!")
    engine.runAndWait()

# === Simulated Ali model to return bounding boxes ===
def get_bounding_boxes_from_ali(frame):
    # Dummy box: (x, y, width, height)
    # Replace with real AI output
    return [(100, 100, 50, 150)]

# === Red color detection inside the bounding boxes ===
def detect_red_band(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    boxes = get_bounding_boxes_from_ali(frame)

    for (x, y, w, h) in boxes:
        roi = hsv[y:y+h, x:x+w]

        # Red color has two HSV ranges
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(roi, lower_red1, upper_red1)
        mask2 = cv2.inRange(roi, lower_red2, upper_red2)
        red_mask = mask1 + mask2

        if cv2.countNonZero(red_mask) > 500:
            return True  # Red band found

    return False  # Red band not found

# === Main function for State 3 ===
def state_3_ali_band(camera):
    print("[STATE 3] Looking for red band...")
    while True:
        ret, frame = camera.read()
        if not ret:
            continue

        found = detect_red_band(frame)

        if found:
            print("✅ Red band found! Going to State 2 (follow).")
            return 'state_2'

        # Not found → scream and keep searching
        scream_behavior()
        time.sleep(1)  # Delay to prevent spam

-------------------------------------------------------------------

Main Block

if __name__ == "__main__":
    cam = cv2.VideoCapture(0)  # Start webcam
    try:
        next_state = state_3_ali_band(cam)
        print(f"➡️ Transition to: {next_state}")
    finally:
        cam.release()
        cv2.destroyAllWindows()

