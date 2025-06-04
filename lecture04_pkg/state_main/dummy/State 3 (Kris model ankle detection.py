State 3 (Kris model: ankle detection)

import cv2
import time
import pyttsx3

# === Setup text-to-speech ===
engine = pyttsx3.init()
engine.setProperty('rate', 150)

def scream_behavior():
    print("😱 AHHHHH!! I LOST THE NURSE!! SEARCHING...")
    engine.say("AHHHHH!! I lost the nurse! Searching!")
    engine.runAndWait()

# === Simulated Kris model: returns bounding boxes for ankles ===
def get_bounding_boxes_from_kris(frame):
    # Simulate detection (e.g., a person is seen)
    # Replace with real model output: return list of (x, y, w, h)
    return []  # Return empty list = not found, to test scream

# === Detect if ankle (person) is in view ===
def detect_ankle(frame):
    boxes = get_bounding_boxes_from_kris(frame)
    return len(boxes) > 0

# === Main function for State 3 (Kris model) ===
def state_3_kris_ankle(camera):
    print("[STATE 3] Looking for ankle using Kris model...")
    while True:
        ret, frame = camera.read()
        if not ret:
            continue

        found = detect_ankle(frame)

        if found:
            print("✅ Ankle (person) found! Going to State 2 (follow).")
            return 'state_2'

        scream_behavior()
        time.sleep(1)

------------------------------------------------------------------------

Main block
if __name__ == "__main__":
    cam = cv2.VideoCapture(0)
    try:
        next_state = state_3_kris_ankle(cam)
        print(f"➡️ Transition to: {next_state}")
    finally:
        cam.release()
        cv2.destroyAllWindows()