import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import numpy as np 
import torch

from utils.trt.common import allocate_buffers
from utils.decorators import timer

class Model:
    def __init__(self, filepath):

        if ".engine" in filepath:
            self.mode = "trt"
            logger = trt.Logger(trt.Logger.VERBOSE)
            runtime = trt.Runtime(logger)

            trt.init_libnvinfer_plugins(logger,'')
            dtype = np.float32

            with open(filepath, "rb") as model:
                self.model = runtime.deserialize_cuda_engine(model.read())

            self.context = self.model.create_execution_context()

            self.inputs, self.outputs, self.bindings, self.stream = allocate_buffers(self.model)

    @timer("Model Time")
    def predict(self, img):

        if self.mode == "trt":

            np.copyto(self.inputs[0].host, img.ravel())

            [cuda.memcpy_htod_async(inp.device, inp.host, self.stream) for inp in self.inputs]
            self.context.execute_async(batch_size=1, bindings=self.bindings, stream_handle=self.stream.handle)
            [cuda.memcpy_dtoh_async(out.host, out.device, self.stream) for out in self.outputs]
            self.stream.synchronize()

            return torch.tensor([out.host for out in self.outputs][3].reshape(1, 25200, 7))
        
