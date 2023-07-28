from scipy.spatial.transform import Rotation as R
import kinematics

def transform_euler(pose, translate, axis, euler, degrees=True):
    """
    使用平移变换加欧拉角对姿态进行变换, 返回新姿态对应的脚尖坐标
    :param pose: 要进行变换的姿态, 六只脚的脚尖坐标
    :param translate: 机体中心平移变换 (x, y, z)
    :param axis: 旋转变换的欧拉角三个轴的顺序 形如 'xyz' 'xy' 
    :param euler: 欧拉角的元组, 顺序与长度要与axis描述一致一致
    :param degrees: 欧拉角单位是否为角度, True为角度, False为弧度
    :return: 新的姿态， 六只脚的新坐标
    """
    quat = R.from_euler(axis, euler, degrees=degrees).as_quat()  # # Build a converter with Euler angles, then convert to quaternion
    # compute each foothold position of the new posture
    pose = tuple(kinematics.transform_pose(leg, pose[leg - 1], translate, quat) for leg in range(1, 7))
    return pose


def transform_quat(pose, translate, quaternion):
    """
    使用平移变换加四元数改变对姿态进行变换
    :param pose: 要进行变换的姿态， 六只脚的脚尖坐标
    :param translate: 机体中心偏移 (x, y, z)
    :param quaternion: 机体的旋转变换四元数 (x, y, z, w)
    :return: 新的姿态， 六只脚的新坐标
    """
    # compute each foothold position of the new posture
    pose = tuple(kinematics.transform_pose(leg, pose[leg - 1], translate, quaternion) for leg in range(1, 7))
    return pose
