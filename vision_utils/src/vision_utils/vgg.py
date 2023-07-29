import cv2
import sys
import os
import pycuda.autoinit
import tensorrt as trt
import pycuda.driver as cuda
import numpy as np


# Simple helper data class that's a little nicer to use than a 2-tuple.
class HostDeviceMem:
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()


def softmax(f):
    f -= np.max(f)
    return np.exp(f) / np.sum(np.exp(f))


def pre_process(img_in, w, h):
    img_in = cv2.resize(img_in, (w, h), interpolation=cv2.INTER_LINEAR)
    img_in = np.transpose(img_in, (2, 0, 1)).astype(np.float32)  # HWC->CHW
    img_in = np.expand_dims(img_in, axis=0)
    img_in /= 255.0
    img_in = np.ascontiguousarray(img_in)
    return img_in


class TrtVGG:
    def __init__(self, model, input_size):
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

    def execute(self, img):
        resized = pre_process(img, self.input_size, self.input_size)
        outputs = self.inference(resized)
        outputs = softmax(outputs[0])
        return outputs

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
