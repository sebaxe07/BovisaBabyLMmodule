from ultralytics import YOLO
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import copy

# Constants
KNOWN_HEIGHT = 1.7  # Average human height (meters)
FOCAL_LENGTH = 500  # Calibrate for your camera
COLORS = [
    (255, 0, 0),    # Red
    (0, 255, 0),    # Green
    (0, 0, 255),    # Blue
    (255, 255, 0),  # Yellow
    (255, 0, 255)   # Magenta
]

# Initialize
model = YOLO("yolo11n_ncnn_model")
tracker = DeepSort(max_age=15)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
ORIGIN_POINT_BIAS = [320, 240]

def cal_x_of_obj(position: []):
    x_center = (position[2] - position[0])/2 - ORIGIN_POINT_BIAS[0]
    y_center = (position[2] - position[0])/2 - ORIGIN_POINT_BIAS[1]
    return x_center


def estimate_distance(box_height):
    return (KNOWN_HEIGHT * FOCAL_LENGTH) / box_height

def to_controller(type, x_position: int, distance: float, human_id: int):
    msg = {'type': type, 'x_position': x_position, 'distance': distance, 'human_id': human_id}
    return msg

def camera():
    ret, frame = cap.read()
    if not ret:
        msg = to_controller('NOTWORKING', 0, 0, 0)
        return msg

    # YOLO Detection
    results = model(frame, classes=[0], conf=0.5)
    detections = []
    if results[0].boxes.size !=0 :
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            h = y2 - y1
            detections.append(([x1, y1, x2-x1, h], float(box.conf[0]), "person"))

        # DeepSORT Tracking
        tracks = tracker.update_tracks(detections, frame=frame)

        min_distance = 10000
        min_ltrb = [1000] * 4
        min_track_id = 1000

        for track in tracks:
            if not track.is_confirmed():
                continue

            try:
                # Ensure track_id is treated as integer
                track_id = int(track.track_id)
                ltrb = track.to_ltrb()

                # Get unique color for this track
                color = COLORS[track_id % len(COLORS)]

                # Calculate distance
                box_height = ltrb[3] - ltrb[1]
                distance = estimate_distance(box_height)
                if min_distance > distance:
                    min_distance = distance
                    min_ltrb = copy.copy(ltrb)
                    min_track_id = track_id


                # Draw bounding box and label
                cv2.rectangle(frame,
                                (int(ltrb[0]), int(ltrb[1])),
                                (int(ltrb[2]), int(ltrb[3])),
                                color, 2)

                cv2.putText(frame,
                            f"Person {track_id} ({distance:.2f}m)",
                            (int(ltrb[0]), int(ltrb[1])-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            except Exception as e:
                print(f"Tracking error: {e}")
                continue

        msg = to_controller('TRACKING', cal_x_of_obj(min_ltrb), min_distance, min_track_id)

    else:
        msg = to_controller('NOTFOUND', 0, 0, 0)


    # cv2.imshow("Human Tracking", frame)
    # if cv2.waitKey(1) == ord('q'):
    #     break
    return msg

cap.release()
cv2.destroyAllWindows()

if __name__ == "__main__":
    camera()