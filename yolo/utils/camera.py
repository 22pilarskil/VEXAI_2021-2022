import pyrealsense2 as rs

def initialize_config(pipeline, device_number):
    config = rs.config()
    config.enable_device(device_number)
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    return config

def switch_cameras(pipeline, oconfig, camera, target_camera=''):
    pipeline.stop()
    try:
        if not target_camera == '':
           config = initialize_config(pipeline, target_camera)
           pipeline.start(config)
           return camera
        if camera == 'd435':
            camera = 'l515'
            config = initialize_config(pipeline, 'f1181409')
            pipeline.start(config)
            return 'l515'
        elif camera == 'l515':
            config = initialize_config(pipeline, '048522072643')
            pipeline.start(config)
            return 'd435'
    except:
        pipeline.start(oconfig)
        return -1

def start_camera(device_number):
    pipeline = rs.pipeline()
    config = initialize_config(pipeline, device_number)
    pipeline.start(config)
    return [pipeline, config]
