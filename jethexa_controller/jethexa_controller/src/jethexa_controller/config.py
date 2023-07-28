import enum
import math


class ServoType(enum.Enum):
    BUS = 1
    PWM = 2


# distribution of servo ID 
#                  \tail/
# id12, id11, id10 |body| id1, id2, id3
# id15, id14, id13 |body| id4, id5, id6
# id18, id17, id16 |body| id7, id8, id9
#                  \head/
#
# Servo name list corresponds to the servo ID. The order is very important!!! 

SERVOS = {
    1: {
        'name': 'coxa_joint_LF',
        'id': 5,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    2: {
        'name': 'femur_joint_LF',
        'id': 3,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    3: {
        'name': 'tibia_joint_LF',
        'id': 1,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    4: {
        'name': 'coxa_joint_LM',
        'id': 11,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    5: {
        'name': 'femur_joint_LM',
        'id': 9,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    6: {
        'name': 'tibia_joint_LM',
        'id': 7,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    7: {
        'name': 'coxa_joint_LR',
        'id': 17,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    8: {
        'name': 'femur_joint_LR',
        'id': 15,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    9: {
        'name': 'tibia_joint_LR',
        'id': 13,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    10: {
        'name': 'coxa_joint_RR',
        'id': 18,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    11: {
        'name': 'femur_joint_RR',
        'id': 16,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': 1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    12: {
        'name': 'tibia_joint_RR',
        'id': 14,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': 1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    13: {
        'name': 'coxa_joint_RM',
        'id': 12,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    14: {
        'name': 'femur_joint_RM',
        'id': 10,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': 1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    15: {
        'name': 'tibia_joint_RM',
        'id': 8,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': 1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    16: {
        'name': 'coxa_joint_RF',
        'id': 6,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': -1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    17: {
        'name': 'femur_joint_RF',
        'id': 4,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': 1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    18: {
        'name': 'tibia_joint_RF',
        'id': 2,
        'center': 500,
        'ticks': 1000,
        'type': ServoType.BUS,
        'direction': 1,
        'max_radians': math.radians(240),
        'offset': math.radians(0),
    },
    19: {
        'name': 'head_pan_joint',
        'id': 1,
        'center': 1500,
        'ticks': 2500 - 500,
        'type': ServoType.PWM,
        'direction': 1,
        'max_radians': math.radians(180),
        'offset': math.radians(0)
    },
    20: {
        'name': 'head_tilt_joint',
        'id': 2,
        'center': 1500,
        'ticks': 2500 - 500,
        'type': ServoType.PWM,
        'direction': 1,
        'max_radians': math.radians(180),
        'offset': math.radians(0)
    },
}

# The definition and distribution of leg id
#      \tile/
# leg4 |body| leg3
# leg5 |body| leg2
# leg6 |body| leg1
#      \head/
#
# Leg name list corresponds to the leg ID. The order is very important.
LEG_LIST = None, "LR", "LM", "LF", "RR", "RM", "RF"
SIMULATE = False
