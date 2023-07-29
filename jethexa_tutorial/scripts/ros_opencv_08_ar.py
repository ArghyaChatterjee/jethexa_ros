#!/usr/bin/python3
# coding=utf8
# Data:2022/03/24
# Author:Aiden
import os
import threading
import cv2
import rospy
import numpy as np
import rospkg
from apriltag import apriltag
from objloader_simple import OBJ as obj_load
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation as R
from vision_utils import fps, point_remapped


OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

AXIS = np.float32([[-1, -1, 0], 
                   [-1,  1, 0], 
                   [ 1,  1, 0], 
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])


MODELS_SCALE = {
                'bicycle': 50, 
                'fox': 4, 
                'chair': 400, 
                'cow': 0.4,
                'wolf': 0.6,
                }
                # 'pirate-ship-fat': 100}


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    cv2.drawContours(img, [imgpts[:4]],-1,(0, 255, 0),-3)
    for i,j in zip(range(4),range(4,8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    cv2.drawContours(img, [imgpts[4:]],-1,(0, 0, 255),3)
    return img


class ARNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        self.camera_intrinsic = np.matrix([[619.063979, 0,          302.560920],
                                           [0,          613.745352, 237.714934],
                                           [0,          0,          1]])

        # 加载 3d 文件
        model_dir = os.path.join(rospkg.RosPack().get_path("jethexa_tutorial"), 'misc/3d_model')
        self.model_name = rospy.get_param("~/model", 'bicycle')
        self.obj = None
        if self.model_name != 'rectangle':
            obj = obj_load(os.path.join(model_dir, self.model_name + '.obj'), swapyz=True)
            obj.faces = obj.faces[::-1]
            new_faces = []
            for face in obj.faces:
                face_vertices = face[0]
                points = []
                colors = []
                for vertex in face_vertices:
                    data = obj.vertices[vertex - 1]
                    points.append(data[:3])
                    if self.model_name != 'cow' and self.model_name != 'wolf':
                        colors.append(data[3:])
                scale_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * MODELS_SCALE[self.model_name]
                points = np.dot(np.array(points), scale_matrix)
                if self.model_name == 'bicycle':
                    points = np.array([[p[0] - 670, p[1] - 350, p[2]] for p in points])
                    points = R.from_euler('xyz', (0, 0, 180), degrees=True).apply(points)
                elif self.model_name == 'fox':
                    points = np.array([[p[0], p[1], p[2]] for p in points])
                    points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                elif self.mode_name == 'chair':
                    points = np.array([[p[0], p[1], p[2]] for p in points])
                    points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                else:
                    points = np.array([[p[0], p[1], p[2]] for p in points])
                if len(colors) > 0:
                    color = tuple(255 * np.array(colors[0]))
                else:
                    color = None
                new_faces.append((points, color))
            self.obj = new_faces

        self.lock = threading.Lock()
        self.fps = fps.FPS()
        self.tag_detector = apriltag("tag36h11")
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(self.camera_rgb_prefix + '/camera_info', CameraInfo, self.camera_info_callback)


    def camera_info_callback(self, msg):
        # rospy.loginfo("image info recv")
        with self.lock:
            self.camera_intrinsic = np.matrix(msg.K).reshape(1, -1, 3)
            self.dist_coeffs = np.array(msg.D)

    def image_callback(self, ros_image: Image):
        #rospy.loginfo("image recv")
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # Convert custom image messages to images
        result_image = np.copy(rgb_image)
        with self.lock:
            try:
                result_image = self.image_proc(rgb_image, result_image)
            except Exception as e:
                rospy.logerr(str(e))
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)
        
    def image_proc(self, rgb_image, result_image):
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        gray = cv2.resize(gray, (320, 180))
        detections = self.tag_detector.detect(gray)
        if detections != ():
            for detection in detections:
                tag_id = detection['id']
                tag_center = detection['center']
                tag_corners = detection['lb-rb-rt-lt']
                # Scale the recognized point coordinates after scaling back to the original image
                lb, rb, rt, lt = [point_remapped(c, (320, 180), (640, 360)) for c in tag_corners[:4]]
                tag_center = point_remapped(tag_center, (320, 180), (640, 360))

                # draw the dots
                cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)
                
                corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                if self.model_name == 'rectangle':
                    imgpts, jac = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                    result_image = draw(result_image, corners, imgpts)
                else:
                     for points, color in self.obj:
                         dst, jac = cv2.projectPoints(points.reshape(-1, 1, 3)/100.0, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                         imgpts = dst.astype(int)
                         # color manually
                         if self.model_name == 'cow':
                             cv2.fillConvexPoly(result_image, imgpts, (0, 255, 255))
                         elif self.model_name == 'wolf':
                             cv2.fillConvexPoly(result_image, imgpts, (255, 255, 0))
                         else:
                             cv2.fillConvexPoly(result_image, imgpts, color)
        return result_image

def main():
    ar_app_node = ARNode('ar_app')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
