import numpy as np
import torch

from utils.decorators import timer, bcolors
from utils.yolo.general import non_max_suppression, scale_coords

try: 
    import pycuda.driver as cuda
    import pycuda.autoinit
    import tensorrt as trt
    from utils.trt.common import allocate_buffers
except ImportError:
    bcolors.print("TRT import fail, check that it is installed or don't use it", 'red')

from models.experimental import attempt_load

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

        if ".pt" in filepath:
            self.mode = "pt"
            self.has_cuda = torch.cuda.is_available()

            self.model = attempt_load(filepath)
            device = torch.device("cuda" if self.has_cuda else 'cpu')
            self.model.to(device)

    @timer("Model Time")
    def predict(self, img, img_shape, conf_thres=.3):
        pred = None

        if self.mode == "trt":

            np.copyto(self.inputs[0].host, img.ravel())

            [cuda.memcpy_htod_async(inp.device, inp.host, self.stream) for inp in self.inputs]
            self.context.execute_async(batch_size=1, bindings=self.bindings, stream_handle=self.stream.handle)
            [cuda.memcpy_dtoh_async(out.host, out.device, self.stream) for out in self.outputs]
            self.stream.synchronize()

            pred = torch.tensor([out.host for out in self.outputs][3].reshape(1, 25200, 7))

        elif self.mode == "pt":
            image_t = None
            if self.has_cuda:
                image_t = torch.cuda.FloatTensor(img)
            else:
                image_t = torch.FloatTensor(img)
            pred = self.model(image_t)[0]


        pred = non_max_suppression(pred, conf_thres=conf_thres)[0]
        pred[:,:4] = scale_coords(img_shape, pred[:,:4], img_shape).round()
        return pred
