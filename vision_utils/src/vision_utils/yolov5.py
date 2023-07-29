import cv2
import sys
import os
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
import numpy as np
import math


# Simple helper data class that's a little nicer to use than a 2-tuple.
class HostDeviceMem:
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()


def sigmoid_v(array):
    return np.reciprocal(np.exp(-array) + 1.0)


def sigmoid(x):
    return 1 / (1 + math.exp(-x))


def non_max_suppression(boxes, confs, classes, iou_thres=0.6):
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = confs.flatten().argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= iou_thres)[0]
        order = order[inds + 1]
    boxes = boxes[keep]
    confs = confs[keep]
    classes = classes[keep]
    return boxes, confs, classes


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def nms(pred, iou_thres=0.4):
    boxes = xywh2xyxy(pred[..., 0:4])
    # best class only
    confs = np.amax(pred[:, 5:], 1, keepdims=True)
    classes = np.argmax(pred[:, 5:], axis=-1)
    return non_max_suppression(boxes, confs, classes, iou_thres)


def make_grid(nx, ny):
    """
    Create scaling tensor based on box location
    Source: https://github.com/ultralytics/yolov5/blob/master/models/yolo.py
    Arguments
        nx: x-axis num boxes
        ny: y-axis num boxes
    Returns
        grid: tensor of shape (1, 1, nx, ny, 80)
    """
    nx_vec = np.arange(nx)
    ny_vec = np.arange(ny)
    yv, xv = np.meshgrid(ny_vec, nx_vec)
    grid = np.stack((yv, xv), axis=2)
    grid = grid.reshape(1, 1, ny, nx, 2)
    return grid


def pre_process(img_in, w, h):
    img_in = cv2.resize(img_in, (w, h), interpolation=cv2.INTER_LINEAR)
    img_in = np.transpose(img_in, (2, 0, 1)).astype(np.float32)
    img_in = np.expand_dims(img_in, axis=0)
    img_in /= 255.0
    img_in = np.ascontiguousarray(img_in)
    return img_in


class TrtYolov5:
    def __init__(self, model, input_size, classes_num):
        # load tensorrt engine
        self.input_size = input_size
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(model, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        self.context = engine.create_execution_context()
        # allocate memory
        inputs, outputs, bindings = [], [], []
        stream = cuda.Stream()
        for binding in engine:
            size = trt.volume(engine.get_binding_shape(binding))
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(device_mem))
            if engine.binding_is_input(binding):
                inputs.append(HostDeviceMem(host_mem, device_mem))
            else:
                outputs.append(HostDeviceMem(host_mem, device_mem))
        # save to class
        self.inputs = inputs
        self.outputs = outputs
        self.bindings = bindings
        self.stream = stream
        # post processing config
        self.strides = np.array([8., 16., 32.])
        anchors = np.array([
            [[10, 13], [16, 30], [33, 23]],
            [[30, 61], [62, 45], [59, 119]],
            [[116, 90], [156, 198], [373, 326]],
        ])
        self.nl = len(anchors)
        self.nc = classes_num  # classes
        self.no = self.nc + 5  # outputs per anchor
        self.na = len(anchors[0])
        a = anchors.copy().astype(np.float32)
        a = a.reshape(self.nl, -1, 2)
        self.anchors = a.copy()
        self.anchor_grid = a.copy().reshape(self.nl, 1, -1, 1, 1, 2)
        self.output_shapes = [
            (1, 3, int(input_size / 8), int(input_size / 8), self.nc + 5),
            (1, 3, int(input_size / 16), int(input_size / 16), self.nc + 5),
            (1, 3, int(input_size / 32), int(input_size / 32), self.nc + 5)
        ]

    def detect(self, img):
        shape_orig_WH = (img.shape[1], img.shape[0])
        resized = pre_process(img, self.input_size, self.input_size)
        outputs = self.inference(resized)
        # reshape from flat to (1, 3, x, y, 85)
        reshaped = []
        for output, shape in zip(outputs, self.output_shapes):
            reshaped.append(output.reshape(shape))
        return reshaped

    def inference(self, img):
        # copy img to input memory
        # self.inputs[0]['host'] = np.ascontiguousarray(img)
        self.inputs[0].host = np.ravel(img)
        # transfer data to the gpu
        [cuda.memcpy_htod_async(inp.device, inp.host, self.stream) for inp in self.inputs]
        # run inference
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        # fetch outputs from gpu
        [cuda.memcpy_dtoh_async(out.host, out.device, self.stream) for out in self.outputs]
        # synchronize stream
        self.stream.synchronize()
        return [out.host for out in self.outputs]

    def post_process(self, image, outputs, conf_thres=0.2, nms_thres=0.4):
        """
        Transforms raw output into boxes, confs, classes
        Applies NMS thresholding on bounding boxes and confs
        Parameters:
            output: raw output tensor
        Returns:
            boxes: x1,y1,x2,y2 tensor (dets, 4)
            confs: class * obj prob tensor (dets, 1)
            classes: class type tensor (dets, 1)
        """
        scaled = []
        grids = []
        for out in outputs:
            out = sigmoid_v(out)
            _, _, width, height, _ = out.shape
            grid = make_grid(width, height)
            grids.append(grid)
            scaled.append(out)
        z = []
        for out, grid, stride, anchor in zip(scaled, grids, self.strides, self.anchor_grid):
            _, _, width, height, _ = out.shape
            out[..., 0:2] = (out[..., 0:2] * 2. - 0.5 + grid) * stride
            out[..., 2:4] = (out[..., 2:4] * 2) ** 2 * anchor

            out = out.reshape((1, 3 * width * height, self.no))
            z.append(out)
        pred = np.concatenate(z, 1)
        xc = pred[..., 4] > conf_thres
        pred = pred[xc]
        return nms(pred, nms_thres)
