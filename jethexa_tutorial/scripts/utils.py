# this file is one part of JetHexa
# /face_detect/utils.py are the tools used in face recognition
# !!!!!!! if you modified this file, you need to execute this file under ～/jethexa directory
# !!!!!!! command colcon build --packages-select face_detect"
# !!!!!!! Rebuild and install the ros package to make this file take effect

import cv2
from vision_utils import point_remapped


def show_faces(detect_img, result_img, boxes, landmarks, bbox_color=(0, 255, 0), ll_color=(0, 0, 255)):
    """Draw bounding boxes and face landmarks on image."""
    detect_size = detect_img.shape[:2]
    show_size = result_img.shape[:2]
    for bb, ll in zip(boxes, landmarks):
        p1 = point_remapped(bb[:2], detect_size, show_size, data_type=int)
        p2 = point_remapped(bb[2:4], detect_size, show_size, data_type=int)
        cv2.rectangle(result_img, p1, p2, bbox_color, 2)
        for i, p in enumerate(ll):
            x, y = point_remapped(p, detect_size, show_size, data_type=int)
            cv2.circle(result_img, (x, y), 2, ll_color, 2)
    return result_img


def mp_face_location(results, img):
    h, w, c, = img.shape
    boxes = []
    keypoints = []
    if results.detections:
        for detection in results.detections:
            x_min = detection.location_data.relative_bounding_box.xmin
            y_min = detection.location_data.relative_bounding_box.ymin
            width = detection.location_data.relative_bounding_box.width
            height = detection.location_data.relative_bounding_box.height
            x_min, y_min = max(x_min * w, 0), max(y_min * h, 0)
            x_max, y_max = min(x_min + width * w, w), min(y_min + height * h, h)
            boxes.append((x_min, y_min, x_max, y_max))
            relative_keypoints = detection.location_data.relative_keypoints
            keypoints.append([(point.x * w, point.y * h) for point in relative_keypoints])
    return boxes, keypoints

