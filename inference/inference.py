import sys
import json
import time
import cv2
from ultralytics import YOLO

def find_largest_person(results):
    """
    Find the largest person in the YOLO detection results.
    Returns a dictionary with person info or None if no person detected.
    """
    # Filter for person class (class 0 in COCO dataset)
    people = [
        box for box in results[0].boxes 
        if int(box.cls[0]) == 0  # Ensure it's a person
    ]
    
    if not people:
        return None
    
    # Find the largest person by area
    largest_person = max(people, key=lambda box: box.xywh[0][2] * box.xywh[0][3])
    
    # Extract center coordinates
    x_center = largest_person.xywh[0][0]
    y_center = largest_person.xywh[0][1]
    
    return {
        "present": True,
        "x": float(x_center),  # Convert to float for JSON serialization
        "y": float(y_center)
    }

def main():
    # Load the YOLO model
    try:
        model = YOLO("yolo11n_ncnn_model", task="detect")  # Using default pretrained model
    except Exception as e:
        print(f"Error loading model: {e}", file=sys.stderr)
        model = None

    if model is None:
        print("Trying to export", file=sys.stderr)
        try:
            model_pt = YOLO("yolo11n.pt", task="detect")
        except Exception as e:
            print(f"Error loading model: {e}", file=sys.stderr)
            return
        model_pt.export(format="ncnn")
        model = YOLO("yolo11n_ncnn_model")
    
    # Open camera
    try:
        for i in range(10):
            cap = cv2.VideoCapture(i)  # Default camera (usually the built-in webcam)
            
            # Check if camera opened successfully
            if cap.isOpened():
                break
            else:
                print(f"Warning: Unable to access camera {i}", file=sys.stderr)
    except Exception as e:
        print(f"Camera initialization error: {e}", file=sys.stderr)
        return
    
    if not cap.isOpened():
        print(f"Error: Unable to access any cameras", file=sys.stderr)
        return

    
    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to capture frame", file=sys.stderr)
                print(json.dumps({"present": False}), flush=True)
                time.sleep(0.1)
                continue
            
            try:
                # Run inference on the frame
                results = model(frame, conf=0.6)
                
                # Find largest person
                person_info = find_largest_person(results)
                
                # Send result to main process
                if person_info:
                    print(json.dumps(person_info), flush=True)
                else:
                    print(json.dumps({"present": False}), flush=True)
            
            except Exception as e:
                print(f"Detection error: {e}", file=sys.stderr)
                print(json.dumps({"present": False}), flush=True)
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.1)
    
    except Exception as e:
        print(f"Subprocess error: {e}", file=sys.stderr)
    
    finally:
        # Ensure camera is released
        if 'cap' in locals():
            cap.release()

if __name__ == '__main__':
    main()