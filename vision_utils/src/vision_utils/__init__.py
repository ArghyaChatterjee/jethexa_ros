import cv2
import math
import numpy as np


def distance(point_1, point_2):
    """
    计算两个点间的距离
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)


def box_center(box):
    """
    计算四边形box的中心
    :param box: box （x1, y1, x2, y2)形式
    :return:  中心坐标（x, y)
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def bgr8_to_jpeg(value, quality=75):
    """
    将cv bgr8格式数据转换为jpg格式
    :param value: 原始数据
    :param quality:  jpg质量 最大值100
    :return:
    """
    return bytes(cv2.imencode('.jpg', value)[1])


def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上
    :param point: 点的坐标
    :param now: 现在图片的尺寸
    :param new: 新的图片尺寸
    :return: 新的点坐标
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)


def get_area_max_contour(contours, threshold=100):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况
    :param contours: 轮廓列表
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None
    """
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None


def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi
    :param v1: 第一个向量
    :param v2: 第二个向量
    :return: 角度
    """
    norm_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (norm_v1_v2)
    sin = np.cross(v1, v2) / (norm_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
    简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平
    可以用在人脸对齐上

    :param image: 要选择的人脸图片
    :param points: 两个点的坐标 ((x1, y1), (x2, y2))
    :param scale: 缩放比例
    :return: 旋转后的图片
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # 计算旋转角度并旋转图片
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))