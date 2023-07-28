import math
from jethexa_controller import kinematics_api

class PoseTransformerParams:
    def __init__(self, **kwargs):
        self.translation = (0, 0, 0)
        self.rotation = (0, 0, 0)
        self.absolutely = False # absolute transformation
        self.duration = 1
        self.__dict__.update(kwargs) # update class member with the input named arguments

def PoseTransformer(params:PoseTransformerParams):
    """
    Use translation transformation plus Euler angles to change the pose of the robot
    :param translate: Translation transformation of body center offset (x, y, z)
    :param rotation: Tuple of Euler angle, 'xyz'
    :param duration: The time taken to complete this transformation
    """
    div_num = max(math.ceil(params.duration / 0.02), 1)# compute how many steps it can be divided into, and there is one step at least. 

    # The total amount of transformation to be performed in this transformation 
    x, y, z = params.translation
    u, v, w = params.rotation


    org_pose, cur_transform = yield None

    # The current transformation of the robot relative to the original
    (org_x, org_y, org_z), (org_u, org_v, org_w) = cur_transform

    if params.absolutely:
        x, y, z = (x - org_x), (y - org_y), (z - org_z)
        u, v, w = (u - org_u), (v - org_v), (w - org_w)
    
        params.translation = x, y, z
        params.rotation = u, v, w

    div_u, div_v, div_w = u / div_num, v / div_num, w / div_num

    # The amount of transformation per small step
    div_x, div_y, div_z = x / div_num, y / div_num, z / div_num
    # the robot transformation relative to the initial, after this transformation
    final_transform =  (org_x + x, org_y + y, org_z + z), (org_u + u, org_v + v, org_w + w)

    # robot posture after this transformation
    final_pose = kinematics_api.transform_euler(org_pose, params.translation, 'xyz', params.rotation, degrees=False)
    
    div_num -= 1
    while div_num >= 0:
        if div_num > 0: # Calculate the transformed position and the transformed absolute transformed value at each step
            # Both translation and Euler angles are linear and can be directly added to the target value in steps by addition
            # the input Euler angle order must be 'xyz'
            nx, ny, nz = x - div_x * div_num, y - div_y * div_num, z - div_z * div_num
            nu, nv, nw = u - div_u * div_num, v - div_v * div_num, w - div_w * div_num

            # the transformation relative to the initial posture (execute the posture set in set_build_in_pose) after completing this step
            out_tranform = (org_x + nx, org_y + ny, org_z + nz), (org_u + nu, org_v + nv, org_w + nw)
            # the posture after completing this step
            cur_pose = kinematics_api.transform_euler(org_pose, (nx, ny, nz), 'xyz', (nu, nv, nw), degrees=False)


            yield cur_pose, out_tranform, False
        else:
            yield final_pose, final_transform, True
        div_num -= 1