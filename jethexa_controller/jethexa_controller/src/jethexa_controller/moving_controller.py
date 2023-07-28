import math
import numpy as np
import kinematics
import rospy

class MovingParams:
    def __init__(self, **kwargs):
        self.forever = False # run permanently
        self.repeat = 1 # repeat times
        self.interrupt = True # whether to stop current movement

        self.gait = 1 # select gait
        self.stride = 0 # stride
        self.height = 0 # lifted height of the leg
        self.direction = 0 # motion direction
        self.rotation = 0 # self-rotation angle of each step
        self.relative_h = False # step height adopts the relative height
        self.linear_factor = 1.0 # The stride coefficient when the robot walks straight, that is, the given stride and the actual stride for one step
        self.rotate_factor = 1.0 # The rotation coefficient when turning
        self.velocity_x = 0 
        self.velocity_y = 0 

        self.period = 10 # interval between each step
        self.__dict__.update(kwargs) # update instance with named parameter
    
    def __str__(self) -> str:
        return str(self.__dict__)

class CmdVelParams:
    def __init__(self, **kwargs):
        self.gait=2,
        self.velocity_x = 0 
        self.velocity_y = 0 
        self.angular_z = 0
        self.height = 10.0
        self.relative_h = False # the step height adopts the relative height

        self.period = 1 # interval between each step

        self.linear_factor = 1.0 # The stride coefficient when the robot walks straight, that is, the given stride and the actual stride for one step
        self.rotate_factor = 1.0 # The rotation coefficient when turning

        self.__dict__.update(kwargs) # update instance with named parameter

    def __str__(self) -> str:
        return str(self.__dict__)


def MovingGenerator(params):
    # compute the actual height according to the relative height or the absolute height
    org_pose = None
    part_index = 0
    sub_index = 0

    sub_action_num = params.period / 6.0 / 0.02 # one step is divided into six strides. Compute how many sub-actions each part has
    sub_action_num = math.ceil(round(max(sub_action_num, 1), 3)) # there is one sub-action in one step at least
    height = params.height
    real_stride = params.stride * params.linear_factor
    real_rotate = params.rotation * params.rotate_factor

    sub_stride = params.stride / (sub_action_num * 6.0)
    sub_rotate = params.rotation / (sub_action_num * 6.0)

    #print("sub_action_num ", params.period, sub_action_num)

    cur_pose = yield None
    while params.repeat > 0 or params.forever:
        org_pose = cur_pose
        if params.relative_h:
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        poses = [] # All postures of the computed gaits
        # Under ripple gait, there is one leg on different direction put down
        # If the leg is put down when the robot stands, the robot body will be supported. Hence this leg should be lifted
        if params.gait == 1:  # ripple gait
            if params.direction >= math.pi:
                start_leg = list(org_pose[0])
                start_leg[2] += height
                start_pose = list(org_pose)
                start_pose[0] = start_leg
            else:
                start_leg = list(org_pose[3])
                start_leg[2] += height
                start_pose = list(org_pose)
                start_pose[3] = start_leg
        else: # not ripple gait
            start_pose = org_pose
        
        # compute the whole gait process
        for i in range(6):
            ps = kinematics.set_step_mode(sub_action_num, 
                                          i, 
                                          start_pose, 
                                          params.gait, 
                                          real_stride,
                                          height, 
                                          params.direction, 
                                          real_rotate)
            ps = np.array(ps)
            # The output data is the multiple poses of each pair of legs, which are converted into the poses of sub-actions of the six legs
            ps = ps.reshape((6, -1, 3))  # convert to [six legs[sub action[coordinate]]] 
            ps = np.transpose(ps, (1, 0, 2))  # convert to [sub action[six legs[coordinate]]] 
            start_pose = ps[-1]
            poses.append(ps)
            
        while params.repeat > 0 or params.forever:
            out_pose = poses[part_index][sub_index]

            # adjust each count 
            sub_index = (sub_index + 1) % sub_action_num # sub-action count +1
            if sub_index == 0:
                part_index = (part_index + 1) % 6 # step count +1
                if part_index == 0:
                    params.repeat = max(params.repeat - 1, 0)
            cur_pose = yield out_pose, part_index == 0 and sub_index == 0, params

            # two initial postures change. We need to compute again.
            if cur_pose is not org_pose:
                break



def CmdVelGenerator(params):
    height = params.height
    org_pose = None
    phase_index = 0 # Current phase cursor
    phase_num = math.ceil(round((params.period * 1000.0 / 20.0), 1)) # Number of subdivided phases
    phase_list = [(i / phase_num) * 2.0 * math.pi for i in range(phase_num)] # Subdivision phase list

    aep_offset_x, aep_offset_y, frac_theta_2_sin, frac_theta_2_cos = kinematics.cmd_vel_basic_data(
        params.velocity_x, #* 1.0, #0.633 ,
        params.velocity_y,
        params.angular_z, #*0.720,
        params.period
    )

    cur_pose = yield None
    while True:
        org_pose = cur_pose
        if params.relative_h:
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        
        aep, pep = kinematics.cmd_vel_aep_pep(
            cur_pose,
            aep_offset_x,
            aep_offset_y,
            frac_theta_2_sin,
            frac_theta_2_cos,
        )

        steps = []
        for phase in phase_list:
            ps = kinematics.cmd_vel_new_point(
                params.gait,
                height,
                phase,
                aep,
                pep
            )
            steps.append(ps)

        while True:
            """
            ps = kinematics.cmd_vel_new_point(
                params.gait,
                height,
                phase_list[phase_index],
                aep,
                pep
            )
            """

            ps = steps[phase_index]
            phase_index =  (phase_index + 1) % phase_num
            if phase_index == 0:
                cur_pose = yield ps, True, params
            else:
                cur_pose = yield ps, False, params
            # two initial postures change. We need to compute again.
            if cur_pose is not org_pose:
                break
