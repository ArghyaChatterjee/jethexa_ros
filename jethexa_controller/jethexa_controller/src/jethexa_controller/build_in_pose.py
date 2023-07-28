X1 = 93.60
Y1 = 50.805
X2 = 0.0
Y2 = 73.535

INITIAL_X = 70.0
INITIAL_Y = 120.0
INITIAL_HEIGHT = 70.0
INITIAL_HEIGHT_M = 95

DEFAULT_POSE = ((INITIAL_X + X1, INITIAL_Y + Y1 - 20, -INITIAL_HEIGHT),\
                (0.0, INITIAL_Y + Y2, -INITIAL_HEIGHT),\
                (-INITIAL_X - X1, INITIAL_Y + Y1 - 20, -INITIAL_HEIGHT),\
                (-INITIAL_X - X1, -INITIAL_Y - Y1 + 20, -INITIAL_HEIGHT),\
                (0.0, -INITIAL_Y - Y2, -INITIAL_HEIGHT),\
                (INITIAL_X + X1, -INITIAL_Y - Y1 + 20, -INITIAL_HEIGHT))

DEFAULT_POSE_M = ((INITIAL_X + X1, INITIAL_Y + Y1 - 20, -INITIAL_HEIGHT_M),\
                (0.0, INITIAL_Y + Y2, -INITIAL_HEIGHT_M),\
                (-INITIAL_X - X1, INITIAL_Y + Y1 - 20, -INITIAL_HEIGHT_M),\
                (-INITIAL_X - X1, -INITIAL_Y - Y1 + 20, -INITIAL_HEIGHT_M),\
                (0.0, -INITIAL_Y - Y2, -INITIAL_HEIGHT_M),\
                (INITIAL_X + X1, -INITIAL_Y - Y1 + 20, -INITIAL_HEIGHT_M))

DEFAULT_POSE_TRANSFORM = (0, 0, 130), (0, 0, 0)
DEFAULT_POSE_M_TRANSFORM = (0, 0, 155), (0, 0, 0)

RELAX_POSE = ((INITIAL_X + X1, INITIAL_Y + Y1, 0.0), (0.0, INITIAL_Y + Y2, 0.0), (-INITIAL_X - X1, INITIAL_Y + Y1, 0.0),
              (-INITIAL_X - X1, -INITIAL_Y - Y1, 0.0), (0.0, -INITIAL_Y - Y2, 0.0), (INITIAL_X + X1, -INITIAL_Y - Y1,
                                                                                     0.0))
SIDE_SHIFT_X = 0
SIDE_SHIFT_Y = 135
SIDE_SHIFT_HEIGHT = 70

SIDE_SHIFT_POSE_TRANSFORM = (0, 0, 130), (0, 0, 0)
SIDE_SHIFT_POSE = ((SIDE_SHIFT_X + X1, SIDE_SHIFT_Y + Y1, -SIDE_SHIFT_HEIGHT),\
                (0.0, SIDE_SHIFT_Y + Y1, -SIDE_SHIFT_HEIGHT),\
                (-SIDE_SHIFT_X - X1, SIDE_SHIFT_Y + Y1, -SIDE_SHIFT_HEIGHT),\
                (-SIDE_SHIFT_X - X1, -SIDE_SHIFT_Y - Y1, -SIDE_SHIFT_HEIGHT),\
                (0.0, -SIDE_SHIFT_Y - Y1, -SIDE_SHIFT_HEIGHT),\
                (SIDE_SHIFT_X + X1, -SIDE_SHIFT_Y - Y1, -SIDE_SHIFT_HEIGHT))

