import os
import re
import csv
import cv2
import time
import rclpy
import numpy as np
import pyrealsense2 as rs
from pynput import keyboard
from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface
from ati_sensor_service.ati_client import AtiClient

# Global flag to track key events
key_pressed = None

# Define the on_press function to capture key events
def on_press(key):
    global key_pressed
    try:
        if key.char == 'o' or key.char == 'O':  # 's' key to start
            key_pressed = 'O'
        elif key.char == 'p' or key.char == 'P':  # 'e' key to stop
            key_pressed = 'P'
    except AttributeError:
        pass  # Ignore special keys


def setup_directories(save_dir, traj_num) -> tuple:
    """Creates directories for storing images and world data."""
    base_folder = os.path.join(save_dir, f"traj_{traj_num}")
    image_folder = os.path.join(base_folder, "images")
    world_folder = os.path.join(base_folder, "world")
    os.makedirs(image_folder, exist_ok=True)
    os.makedirs(world_folder, exist_ok=True)
    return image_folder, world_folder, os.path.join(base_folder, "waypoints.csv")


def save_image(folder, index, image):
    """Save an image with a given index in the specified folder."""
    filename = os.path.join(folder, f"{index + 1}.png")
    cv2.imwrite(filename, image)

def get_color_frame(pipeline):
    color_frame = None
    try:
        frames1 = pipeline.wait_for_frames(timeout_ms=1000)  # Add a timeout to avoid infinite waiting
        color_frame = frames1.get_color_frame()
    except Exception as e:
        print(f"Error retrieving frames: {e}")
    finally:
        return color_frame


def show_color_frame(color_frame):
    color_image = np.asanyarray(color_frame.get_data())
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)
    return color_image

def main(save_dir, traj_num):
    
    global key_pressed
    rclpy.init()

    ###############################
    #------ Create Clients -------#
    ###############################

    kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
                         move_group_name="kuka_blue", 
                         remapping_name="kuka_blue",  
                         prefix="",                   
                         )

    # Create two pipelines for two cameras
    # pipeline1 = rs.pipeline() #IMG_CAPTURE
    # config1 = rs.config() #IMG_CAPTURE
    # config1.enable_device('038522062901')  #IMG_CAPTURE
    # config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # IMG_CAPTURE
    # pipeline1.start(config1) #IMG_CAPTURE

    # Configure the second pipeline (camera 2)
    # pipeline2 = rs.pipeline()
    # config2 = rs.config()
    # config2.enable_device('932122060300')  # Replace with the serial number of your second camera
    # config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed
    # pipeline2.start(config2)

    ati_client = AtiClient()

    # Track whether we are currently saving data
    is_writing = False
    writer = None
    trajectory_file = None
    row_index = 0  # Start index for the image file names

    if os.listdir(save_dir):
        files = sorted(os.listdir(save_dir), key=lambda x: int(re.search(r'(\d+)', x).group()))
        num = files[-1].split('_')[-1]
        traj_num = int(num) + 1

    header = ['Time', 'J1','J2','J3','J4','J5','J6', 'J7', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    # Set up the key listener
    print("Instructions Press 'O' to start recording data, 'P' to stop.")
    with keyboard.Listener(on_press=on_press) as listener:
        while True:
            if key_pressed == 'O':
                LISTNER_START_TIME = time.time()
                if not is_writing:
                    print(f"Run number - {traj_num}")
                    # Create directory for saving images if it doesn't exist
                    image_folder, world_folder, trajectory_file = setup_directories(save_dir, traj_num)
                    # Open the file outside of the loop and keep it open for writing
                    trajectory_file_obj = open(trajectory_file, mode='w', newline='')
                    writer = csv.writer(trajectory_file_obj)
                    writer.writerow(header)
                    is_writing = True  # We are now writing data
                    row_index = 0  # Reset the row index for the new file
                    print(f"Started writing to {trajectory_file}")
                    traj_num += 1  # Increment the trajectory number for the next file
                    key_pressed = None  # Reset the key event

            elif key_pressed == 'P':
                if is_writing:
                    print(f"Stopping data collection. File {trajectory_file} saved.")
                    trajectory_file_obj.close()  # Explicitly close the file when stopping
                    is_writing = False  # Stop writing data
                    trajectory_file = None
                    key_pressed = None  # Reset the key event

            # If we are writing data, perform the usual loop
            if is_writing:
                
                TICK_TIME = time.time()  # Record the start time of the loop iteration
                
                # Get the current joint state
                action_observation = kb.get_current_joint_state()
                # Get the force torque data
                force_observations = ati_client.get_force_torque()
                rclpy.spin_until_future_complete(ati_client, force_observations)

                # color_frame1 = get_color_frame(pipeline1)   #IMG_CAPTURE

                OBSERVATION_TIME = time.time() - LISTNER_START_TIME

                # if color_frame1: image_observation1 = show_color_frame(color_frame1) #IMG_CAPTURE

                if action_observation and force_observations.done():
                    # Create the row for the CSV
                    response_kuka = [OBSERVATION_TIME]

                    response_kuka.extend(action_observation.position)
                    
                    # print(f'Mocap Service call completed {response_kuka}')

                    response_ati = force_observations.result().msg
                    
                    # ati_client.get_logger().info(f"Ati Service call successful {response_ati}")
                    force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
                    response_kuka.extend(force_values)

                    # Write the data to the CSV file
                    if writer:  # Check if the writer is not None
                        writer.writerow(response_kuka)

                    # Save the image with the corresponding row index
                    # save_image(image_folder, row_index, image_observation1) #IMG_CAPTURE

                    row_index += 1  # Increment the index for the next image

                # Calculate the time taken for the current iteration
                TOCK_TIME = time.time()
                _ELAPSED_TIME = TOCK_TIME - TICK_TIME
                
                _SAMPLING_TIME = 0.0333333  # 30 Hz sampling time

                if _ELAPSED_TIME > _SAMPLING_TIME:
                    print(f"Loop iteration took {_ELAPSED_TIME} seconds. Sampling time exceeded.")
                else:
                    time.sleep(_SAMPLING_TIME - _ELAPSED_TIME) 

if __name__ == '__main__':
    main('no-sync/replay_traj_data', 1)
