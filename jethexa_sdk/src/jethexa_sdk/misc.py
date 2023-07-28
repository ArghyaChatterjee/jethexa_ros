
def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def empty_func(img=None):
    return img


def set_range(x, x_min, x_max):
    tmp = x if x > x_min else x_min
    tmp = tmp if tmp < x_max else x_max
    return tmp
