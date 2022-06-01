import time

from cv_bridge import CvBridge

from lane_detection.lane_detection import detect_lanes


pub = None

def set_publisher(publisher):
    global pub
    pub = publisher

last_time = time.time()
def get_lane_value(img):
    global last_time
    FPS = (time.time() - last_time) * 1000
    
    detection = detect_lanes(img)
    img = detection["image"]
    img_presp = detection["image_prespective"]
    if pub is not None:
        ros_img = CvBridge().cv2_to_imgmsg(img_presp, encoding="bgr8")
        pub.publish(ros_img)
    # time.sleep(0.12)
    last_time = time.time()
    return {
        "left_lane": detection["left_lane"],
        "right_lane": detection["right_lane"],
        "exit_lane": detection["exit_lane"],
        "FPS": FPS
    }
