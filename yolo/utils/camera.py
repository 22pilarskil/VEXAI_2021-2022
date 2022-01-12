import pyrealsense2 as rs

def initialize_config(pipeline, device_number):

    config = rs.config()
    config.enable_device(device_number)
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device().first_depth_sensor()
    device.set_option(rs.option.min_distance, 0)
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        print("here")
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

def switch_cameras(pipeline, oconfig, camera):
    pipeline.stop()
    try:
        config = initialize_config(pipeline, camera)
    except:
        pipeline.start(oconfig)
        return -1
