import os
import re
import csv
import cv2
import time
import json
import rclpy
import copy
import math
import numpy as np
import pyrealsense2 as rs
import depthai as dai
from pynput import keyboard
from concurrent.futures import ThreadPoolExecutor
from ati_sensor_service.ati_client import AtiClient
from mocap_service.mocap_service_client import MocapClient
from rclpy.node import Node

# Global flag to track key events
key_pressed = None

STREAM_TYPE_MAP = {
    "color": rs.stream.color,
    "depth": rs.stream.depth,
    "infrared": rs.stream.infrared
}



# Define the on_press function to capture key events
def on_press(key):
    global key_pressed
    try:
        if key.char == 'o' or key.char == 'O':  # Start recording data
            key_pressed = 'O'
        elif key.char == 'p' or key.char == 'P':  # Stop recording data
            key_pressed = 'P'
    except AttributeError:
        pass  # Ignore special keys
 

# Function to get serial numbers of connected RealSense devices
def get_serial_numbers():
    context = rs.context()
    serial_numbers = []
    print(f"Number of connected devices: {len(context.devices)} Realsense cameras")
    for device in context.devices:
        serial_number = device.get_info(rs.camera_info.serial_number)
        serial_numbers.append(serial_number)
        print(f"Found device with serial number: {serial_number}")
    return serial_numbers


# Function to set up a pipeline for each camera based on its serial number
def setup_pipeline(serial_number):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial_number)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    return pipeline

def setup_oak_pipeline():
    """Set up OAK-D pipeline with alignment"""
    pipeline = dai.Pipeline()

    # Color camera
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)

    # Mono cameras
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Stereo depth
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)

    # Alignment
    align = pipeline.create(dai.node.ImageAlign)
    align.setOutputSize(640, 480)
    align.setInterpolation(dai.Interpolation.BILINEAR)

    # XLink outputs
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_aligned = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_aligned.setStreamName("aligned")

    # Linking
    cam_rgb.preview.link(xout_rgb.input)
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(align.input)
    cam_rgb.video.link(align.inputAlignTo)
    align.outputAligned.link(xout_aligned.input)

    print("OAK-D pipeline set up successfully.")

    return pipeline

def save_oak_calibration(device, cam_folder):
    """Get OAK-D intrinsics in RealSense-compatible format"""
    calib = device.readCalibration()
    
    # Get RGB camera intrinsics (aligned depth perspective)
    rgb_cam = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 640, 400)
    
    # Get left camera intrinsics (original depth perspective)
    left_intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)
    
    # Get critical stereo baseline (convert cm to meters)
    baseline = calib.getBaselineDistance(
        dai.CameraBoardSocket.CAM_B, 
        dai.CameraBoardSocket.CAM_C
    ) * 0.01

    # Create calibration structure
    calibration_data = {
        "color_intrinsics": {
            "width": 640,
            "height": 480,
            "ppx": rgb_cam[0][2],    # [0][2] of 3x3 matrix
            "ppy": rgb_cam[1][2],    # [1][2] of 3x3 matrix
            "fx": rgb_cam[0][0],     # [0][0] of 3x3 matrix
            "fy": rgb_cam[1][1],     # [1][1] of 3x3 matrix
            "coeffs": [0,0,0,0,0]
        },
        "depth_intrinsics": {
            "width": 640,            # From mono camera resolution
            "height": 480,           # From mono camera resolution
            "ppx": left_intr[0][2],   # [0][2] of 3x3 matrix
            "ppy": left_intr[1][2],   # [1][2] of 3x3 matrix
            "fx": left_intr[0][0],   # [0][0] of 3x3 matrix
            "fy": left_intr[1][1],   # [1][1] of 3x3 matrix
            "baseline": baseline,
            "depth_scale": 0.001
        }
    }

    print("✅ Got Color Intrinsics:", calibration_data["color_intrinsics"])
    print("✅ Got Depth Intrinsics:", calibration_data["depth_intrinsics"])

    # Save calibration file
    calibration_file = os.path.join(cam_folder, "calibration_oak_d.json")
    with open(calibration_file, 'w') as f:
        json.dump(calibration_data, f, indent=4)


# Function to stop all pipelines
def stop_pipelines(pipelines):
    for pipeline in pipelines:
        pipeline.stop()


# Function to capture frames from each camera pipeline
def capture_frames(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    frames = (np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()))
    return frames

def save_as_binary(image, filepath):
    image.tofile(filepath)

# Function to save RGB and Depth images to specific folders
def save_images_parallel(frames_dict, base_folder, traj_number, timestamp):
    """Save RGB/depth images with built-in depth conversion"""
    def preprocess_depth(depth_image):
        """Embedded depth conversion (thread-safe)"""
        if depth_image.dtype != np.uint8:
            return cv2.convertScaleAbs(depth_image, alpha=(255.0/depth_image.max()))
        return depth_image

    def save_single_camera(cam_id, color_img, depth_img, timestamp):
        """Thread-safe per-camera save operation"""
        #  Generate filenames
        rgb_path = f"{base_folder}/camera_{cam_id+1}/rgb/rgb_{traj_number}_{cam_id+1}_{timestamp}.png"
        depth_path = f"{base_folder}/camera_{cam_id+1}/depth/depth_{traj_number}_{cam_id+1}_{timestamp}.png"

        # Convert depth before saving
        processed_depth = preprocess_depth(depth_img)
        
        # Save images
        cv2.imwrite(rgb_path, color_img)
        cv2.imwrite(depth_path, processed_depth)

    # Process all cameras in parallel
    with ThreadPoolExecutor() as executor:
        futures = []
        for cam_id, (color_img, depth_img) in frames_dict.items():
            futures.append(
                executor.submit(save_single_camera, cam_id, color_img, depth_img, timestamp)
            )
        
        # Optional: Wait for all saves to complete
        for future in futures:
            future.result()  # Raises exceptions if any occurred


# Function to get the intrinsics for the RGB camera
def get_intrinsics(sensor, stream_type_str):
    """
    Get intrinsics for the given stream type ("color", "depth", "infrared").
    """
    stream_type = STREAM_TYPE_MAP.get(stream_type_str.lower())
    if stream_type is None:
        raise ValueError(f"Unsupported stream type: {stream_type_str}")

    for profile in sensor.get_stream_profiles():
        if profile.stream_type() == stream_type and profile.is_video_stream_profile():
            video_profile = profile.as_video_stream_profile()
            intr = video_profile.get_intrinsics()

            intrinsics = {
                "width": 640,
                "height": 480,
                "ppx": intr.ppx,
                "ppy": intr.ppy,
                "fx": intr.fx,
                "fy": intr.fy,
                "k1": intr.coeffs[0],
                "k2": intr.coeffs[1],
                "p1": intr.coeffs[2],
                "p2": intr.coeffs[3],
                "k3": intr.coeffs[4]
            }

            # If this is a depth stream, try to include depth scale
            if stream_type_str.lower() == "depth" and sensor.supports(rs.option.depth_units):
                depth_scale = sensor.get_option(rs.option.depth_units)
                intrinsics["depth_scale"] = depth_scale

            return intrinsics

    raise ValueError(f"No suitable {stream_type_str} video stream profile found.")


# Function to save calibration data (intrinsics and extrinsics)
def save_camera_calibration(pipeline, camera_idx, base_folder, serial_number):
    camera_folder = os.path.join(base_folder, f"camera_{camera_idx + 1}")
    os.makedirs(camera_folder, exist_ok=True)
    
    device = pipeline.get_active_profile().get_device()
    sensors = device.query_sensors()

    # Try to get color and depth intrinsics
    color_intrinsics = None
    depth_intrinsics = None

    for sensor in sensors:
        try:
            if color_intrinsics is None:
                color_intrinsics = get_intrinsics(sensor, "color")
                print("✅ Got Color Intrinsics:", color_intrinsics)
        except Exception as e:
            pass  # Not this sensor

        try:
            if depth_intrinsics is None:
                depth_intrinsics = get_intrinsics(sensor, "depth")
                print("✅ Got Depth Intrinsics:", depth_intrinsics)
        except Exception as e:
            pass  # Not this sensor

    if not color_intrinsics:
        print("⚠️  Color intrinsics not found.")
    if not depth_intrinsics:
        print("⚠️  Depth intrinsics not found.")

    # Get extrinsics (between color and depth streams)
    # extrinsics = get_extrinsics_to(color_stream, depth_stream)

    # Save to a JSON file
    calibration_data = {
        "color_intrinsics": color_intrinsics,
        "depth_intrinsics" : depth_intrinsics,
    }

    calibration_file = os.path.join(camera_folder, f"calibration_{serial_number}.json")
    with open(calibration_file, 'w') as f:
        json.dump(calibration_data, f, indent=4)

def process_mocap_data(mocap_response):
    """Processes mocap data and returns structured results with header info."""
    # Initialize containers
    header = ['Time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']
    data = {}
    rb_ids = set()
    ms_ids = set()

    # Extract rigid bodies
    rigid_bodies = {}
    if mocap_response.latest_message.rigid_bodies:
        for rb in mocap_response.latest_message.rigid_bodies:
            rb_id = rb.id
            rb_ids.add(rb_id)
            rigid_bodies[rb_id] = [
                rb.pose_stamped.pose.position.x,
                rb.pose_stamped.pose.position.y,
                rb.pose_stamped.pose.position.z,
                rb.pose_stamped.pose.orientation.w,
                rb.pose_stamped.pose.orientation.x,
                rb.pose_stamped.pose.orientation.y,
                rb.pose_stamped.pose.orientation.z
            ]

    # Extract marker sets
    # marker_sets = {}
    # if mocap_response.latest_message.marker_sets:
    #     for ms in mocap_response.latest_message.marker_sets:
    #         ms_id = ms.id
    #         ms_ids.add(ms_id)
    #         marker_sets[ms_id] = [
    #             ms.position.x,
    #             ms.position.y,
    #             ms.position.z
    #         ]

    # Build header once (sorted for consistency)
    for rb_id in sorted(rb_ids):
        header.extend([
            f"rb_{rb_id}_x", f"rb_{rb_id}_y", f"rb_{rb_id}_z",
            f"rb_{rb_id}_qw", f"rb_{rb_id}_qx", f"rb_{rb_id}_qy", f"rb_{rb_id}_qz"
        ])
    
    # for ms_id in sorted(ms_ids):
    #     header.extend([
    #         f"ms_{ms_id}_x", f"ms_{ms_id}_y", f"ms_{ms_id}_z"
    #     ])

    # Structure data with None placeholders for missing entries
    data = {
        'rigid_bodies': rigid_bodies,
        # 'marker_sets': marker_sets,
        'header': header
    }

    return data

# Function to find the next available trajectory folder
def get_next_traj_folder(base_folder="traj"):
    traj_number = 1
    # Check existing traj folders and increment the number
    while os.path.exists(f"{base_folder}{traj_number}"):
        traj_number += 1
    return f"{base_folder}{traj_number}"


def display_frames(frames_dict):
    for i, (color_img, _) in frames_dict.items():
        cv2.imshow(f"Camera {i+1} RGB", color_img)

def display_frames_single_view(frames_dict, window_name="All Cameras"):
    # Extract only the color images
    frames = [frame[0] for frame in frames_dict.values()]
    
    if not frames:
        return

    # Resize all frames to same size (use first frame as reference)
    h, w = frames[0].shape[:2]
    resized_frames = [cv2.resize(f, (w, h)) for f in frames]

    # Determine grid layout (auto square-ish)
    N = len(resized_frames)
    cols = math.ceil(math.sqrt(N))
    rows = math.ceil(N / cols)

    # Pad frames if needed to fill the grid
    while len(resized_frames) < rows * cols:
        blank = np.zeros_like(resized_frames[0])
        resized_frames.append(blank)

    # Stack frames row by row
    rows_imgs = [
        np.hstack(resized_frames[i*cols:(i+1)*cols])
        for i in range(rows)
    ]
    grid_img = np.vstack(rows_imgs)

    # Show the combined image
    cv2.imshow(window_name, grid_img)


# Main function to orchestrate data collection and RealSense integration
def main(save_dir="no-sync/rgbd_force_motive_data"):
    
    global key_pressed

    rclpy.init()

    ###############################
    #------ Create Clients -------#
    ###############################

    ati_client = AtiClient()

    mocap_client = MocapClient()

    # Get serial numbers of connected RealSense devices and set up pipelines
    serial_numbers = get_serial_numbers()
    
    if not serial_numbers:
        print("Error: No RealSense cameras detected.")
        return
    

    #### relsesense pipeline
    pipelines = [setup_pipeline(sn) for sn in serial_numbers]

    # OAK-D setup
    oak_pipeline = setup_oak_pipeline()
    oak_device = dai.Device(oak_pipeline)
    oak_rgb_queue = oak_device.getOutputQueue("rgb", 4, False)
    oak_depth_queue = oak_device.getOutputQueue("aligned", 4, False)

    if len(pipelines) < 3:
        print("Error: Not enough realsense cameras connected")
        return
    if oak_pipeline is None:
        print("Error: OAK-D pipeline not set up correctly")
        return

    ## Total number of cameras
    total_cams = len(serial_numbers) + 1
    
    print("Instructions: Press 'O' to start recording data and 'P' to stop.")

    is_writing = False
    writer = None
    trajectory_file_obj = None
    traj_folder = None
    trajectory_file = None

    with keyboard.Listener(on_press=on_press) as listener:

        while True:
            if key_pressed == 'O'and not is_writing:
                    TICK_TIME = time.monotonic() 
                    
                    traj_folder = get_next_traj_folder(f"{save_dir}/traj")
                    print(f"Recording to: {traj_folder}")
                    os.makedirs(traj_folder, exist_ok=True)

                    for idx in range(total_cams):
                        cam_folder = os.path.join(traj_folder, f"camera_{idx+1}")
                        os.makedirs(os.path.join(cam_folder, "rgb"), exist_ok=True)
                        os.makedirs(os.path.join(cam_folder, "depth"), exist_ok=True)

                        if idx == total_cams-1:  # OAK-D
                            save_oak_calibration(oak_device, cam_folder)
                        else:
                            save_camera_calibration(pipelines[idx], idx, traj_folder, serial_numbers[idx])

                    trajectory_file = os.path.join(traj_folder, "waypoints.csv")
                    trajectory_file_obj = open(trajectory_file, mode='w', newline='')
                    writer = csv.writer(trajectory_file_obj)
                    is_writing = True  # Start writing data
                    print(f"Started writing to {trajectory_file}")
                    header_done = False

            elif key_pressed == 'P':
                if is_writing:
                    print(f"Stopping data collection. File {trajectory_file} saved.")
                    trajectory_file_obj.close()  # Explicitly close file when stopping
                    is_writing = False  # Stop writing data

            frames_dict = {}
            # Capture frames from each RealSense camera
            for i, pipeline in enumerate(pipelines):
                frames_dict[i] = capture_frames(pipeline)

            # Capture frames from OAK-D
            oak_rgb = oak_rgb_queue.get().getCvFrame()
            oak_depth = oak_depth_queue.get().getCvFrame()

            # Combine OAK-D frames with RealSense frames
            frames_dict[total_cams-1] = (oak_rgb, oak_depth)

            # Display frames
            display_frames_single_view(frames_dict)


            if is_writing:
                start_time = time.monotonic()   # Record start time of loop iteration
                
                force_observations_future = ati_client.get_force_torque()
                rclpy.spin_until_future_complete(ati_client, force_observations_future)

                state_observations_future = mocap_client.send_request()
                rclpy.spin_until_future_complete(mocap_client, state_observations_future)


                if force_observations_future.done() and state_observations_future.done() and frames_dict:

                    TOCK_TIME = time.monotonic()  - TICK_TIME

                    response_ati_msg = force_observations_future.result().msg
                    values = [TOCK_TIME,
                            response_ati_msg.fx,
                            response_ati_msg.fy,
                            response_ati_msg.fz,
                            response_ati_msg.tx,
                            response_ati_msg.ty,
                            response_ati_msg.tz]
                    

                    response_mocap_msg = state_observations_future.result()
                    mocap_data = process_mocap_data(response_mocap_msg)

                    # Add rigid bodies in sorted order
                    for rb_id in sorted(mocap_data['rigid_bodies'].keys()):
                        values.extend(mocap_data['rigid_bodies'].get(rb_id, [None]*7))
                    
                    # Add marker sets in sorted order
                    # for ms_id in sorted(mocap_data['marker_sets'].keys()):
                    #     values.extend(mocap_data['marker_sets'].get(ms_id, [None]*3))
                    
                    if header_done == False:
                        writer.writerow(mocap_data['header'])
                        header_done = True

                    frames_dict_copy = copy.deepcopy(frames_dict)

                    # print(f"Writing data: {force_values + mocap_data['rigid_bodies'] + mocap_data['marker_sets']}")
                    save_images_parallel(frames_dict_copy, traj_folder, traj_number=traj_folder[-1], timestamp=TOCK_TIME)
                    writer.writerow(values)
                    # Save RGB and depth images

                elapsed_time = time.monotonic()  - start_time
                sleep_time = 0.033333 - elapsed_time  # Ensure the loop runs at approximately 0.03 sec intervals

                if sleep_time > 0:
                    time.sleep(sleep_time)

            cv2.waitKey(1)
            # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
