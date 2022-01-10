import tensorrt as trt
TRT_LOGGER = trt.Logger(trt.Logger.VERBOSE)
trt.init_libnvinfer_plugins(TRT_LOGGER,'')
DTYPE_TRT = trt.float32
import pycuda.driver as cuda
import pycuda.autoinit
from PIL import Image
import numpy as np
path_img = "0.jpg"
import common

# ...
# some helper functions from TRT sample Python code
# ...



def load_input(img_path, host_buffer):
    # convert to BGR and CHW format
    with Image.open(img_path) as img:
        # RGB to BGR
        r, g, b = img.split()              
        img = Image.merge('RGB', (b, g, r))

        c, h, w = (3, 640, 640)
        dtype = trt.nptype(DTYPE_TRT) 
        img_res = img.resize((w, h), Image.BICUBIC)
        img_res = np.array(img_res, dtype=dtype, order='C')

        # HWC to CHW format:
        img_chw = np.transpose(img_res, [2, 0, 1])
       

        img_array = img_chw.ravel()
        np.copyto(host_buffer, img_array)

# Inference
with open("best.trt", "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
    engine = runtime.deserialize_cuda_engine(f.read())
    
    with engine.create_execution_context() as context:

        # allocate buffers
        inputs, outputs, bindings, stream = common.allocate_buffers(engine)
        #stream = cuda.Stream()

        # load image and pre-processing
        load_input(path_img, inputs[0].host)

        # transfer input data to the GPU.
        #cuda.memcpy_htod_async(inputs[0].device, inputs[0].host, stream)
        [cuda.memcpy_htod_async(inp.device, inp.host, stream) for inp in inputs]
        
        # inference
        inference = context.execute_async(batch_size=1, bindings=bindings, stream_handle=stream.handle)
        
        # Transfer predictions back from the GPU.
        #cuda.memcpy_dtoh_async(outputs[0].host, outputs[0].device, stream)
        [cuda.memcpy_dtoh_async(out.host, out.device, stream) for out in outputs]
        
        # Synchronize the stream
        stream.synchronize()
        
        # Print the host output:
        #trt_output = outputs[0].host
        
        trt_output = [out.host for out in outputs]
        
        print("OUTPUT")
        #for i in range(0, len(trt_output)):
            #print(trt_output[i].shape)
        print(trt_output[3].shape)

        trt_output = np.reshape(trt_output[3], (25200,7))#final yolo output is 1x25200x7
        print(trt_output.shape)

        
