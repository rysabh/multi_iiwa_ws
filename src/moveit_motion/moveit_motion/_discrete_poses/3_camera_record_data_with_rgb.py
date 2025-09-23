import os
import re
import csv
import cv2
import time
import json
import rclpy
import numpy as np
import pyrealsense2 as rs
from pynput import keyboard
from ati_sensor_service.ati_client import AtiClient

# Global flag to track key events
key_pressed = None

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
    for device in context.devices:
        serial_number = device.get_info(rs.camera_info.serial_number)
        serial_numbers.append(serial_number)
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


# Function to stop all pipelines
def stop_pipelines(pipelines):
    for pipeline in pipelines:
        pipeline.stop()


# Function to capture frames from each camera pipeline
def capture_frames(pipelines):
    frames_dict = {}
    for i, pipeline in enumerate(pipelines):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        frames_dict[i] = (np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()))
    return frames_dict


# Function to save RGB and Depth images to specific folders
def save_images(frames_dict, base_folder, traj_number):
    for i, (color_image, depth_image) in frames_dict.items():
        # Create directories for camera_X/rgb and camera_X/depth
        camera_folder = os.path.join(base_folder, f"camera_{i + 1}")
        rgb_folder = os.path.join(camera_folder, "rgb")
        depth_folder = os.path.join(camera_folder, "depth")

        # Create the directories if they don't exist
        os.makedirs(rgb_folder, exist_ok=True)
        os.makedirs(depth_folder, exist_ok=True)

        # Save RGB and depth images with filenames including trajectory number and timestamp
        timestamp = int(time.time())  # Current timestamp for unique filenames
        rgb_filename = os.path.join(rgb_folder, f"rgb_{traj_number}_{i + 1}_{timestamp}.png")
        depth_filename = os.path.join(depth_folder, f"depth_{traj_number}_{i + 1}_{timestamp}.png")

        # Save the images using OpenCV
        cv2.imwrite(rgb_filename, color_image)
        cv2.imwrite(depth_filename, depth_image)


# Function to set up directories for saving data
def setup_directories(save_dir, traj_num) -> tuple:
    """Creates directories for storing images and world data."""
    base_folder = os.path.join(save_dir, f"traj_{traj_num}")
    image_folder = os.path.join(base_folder, "images")
    os.makedirs(image_folder, exist_ok=True)
    return image_folder, os.path.join(base_folder, "waypoints.csv")

# Main function to orchestrate data collection and RealSense integration
def main(save_dir="no-sync/replay_traj_data"):
    
    global key_pressed

    rclpy.init()

    ###############################
    #------ Create Clients -------#
    ###############################

    ati_client = AtiClient()

    # Get serial numbers of connected RealSense devices and set up pipelines
    serial_numbers = get_serial_numbers()
    
    if not serial_numbers:
        print("Error: No RealSense cameras detected.")
        return
    
    pipelines = [setup_pipeline(sn) for sn in serial_numbers]
    
    print("Instructions: Press 'O' to start recording data and 'P' to stop.")

    is_writing = False
    writer = None
    trajectory_file_obj = None
    row_index = 0  # Start index for image file names

    traj_num = 1

    if os.listdir(save_dir):
        files = sorted(os.listdir(save_dir), key=lambda x: int(re.search(r'(\d+)', x).group()))
        num = files[-1].split('_')[-1]
        traj_num = int(num) + 1

    header = ['Time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    with keyboard.Listener(on_press=on_press) as listener:
        
        try:
            while True:
                if key_pressed == 'O':
                    LISTNER_START_TIME = time.time()
                    if not is_writing:
                        print(f"Run number - {traj_num}")
                        image_folder, trajectory_file = setup_directories(save_dir, traj_num)
                        trajectory_file_obj = open(trajectory_file, mode='w', newline='')
                        writer = csv.writer(trajectory_file_obj)
                        writer.writerow(header)
                        is_writing = True  # Start writing data
                        row_index = 0  # Reset row index for new file
                        print(f"Started writing to {trajectory_file}")
                        traj_num += 1

                elif key_pressed == 'P':
                    if is_writing:
                        print(f"Stopping data collection. File {trajectory_file} saved.")
                        trajectory_file_obj.close()  # Explicitly close file when stopping
                        is_writing = False  # Stop writing data

                if is_writing:
                    TICK_TIME = time.time()  # Record start time of loop iteration
                    
                    force_observations_future = ati_client.get_force_torque()
                    rclpy.spin_until_future_complete(ati_client, force_observations_future)

                    OBSERVATION_TIME = time.time() - LISTNER_START_TIME
                    
                    if force_observations_future.done():
                        response_ati_msg = force_observations_future.result().msg
                        force_values = [OBSERVATION_TIME,
                                        response_ati_msg.fx,
                                        response_ati_msg.fy,
                                        response_ati_msg.fz,
                                        response_ati_msg.tx,
                                        response_ati_msg.ty,
                                        response_ati_msg.tz]
                        
                        writer.writerow(force_values)

                    frames_dict = capture_frames(pipelines)
                    save_images(frames_dict, image_folder, traj_num)

                    TOCK_TIME = time.time()
                    _ELAPSED_TIME = TOCK_TIME - TICK_TIME
                    
                    _SAMPLING_TIME = 0.0333333  # Sampling time (30 Hz)

                    if _ELAPSED_TIME > _SAMPLING_TIME:
                        print(f"Loop iteration took {_ELAPSED_TIME} seconds. Sampling time exceeded.")
                    else:
                        time.sleep(_SAMPLING_TIME - _ELAPSED_TIME)

                # Display RGB images from all cameras in real-time.
                frames_dict_displayed = capture_frames(pipelines)
                for i, (color_image_displayed, _) in frames_dict_displayed.items():
                    cv2.imshow(f"Camera {i + 1} RGB", color_image_displayed)

                if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit visualization.
                    break
        
        finally:
            stop_pipelines(pipelines)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
