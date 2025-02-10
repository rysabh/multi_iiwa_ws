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
    pipeline1 = rs.pipeline()
    # pipeline2 = rs.pipeline()

    # Configure the first pipeline (camera 1)
    config1 = rs.config()
    config1.enable_device('038522062901')  # Replace with the actual serial number of your first camera
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed

    # Configure the second pipeline (camera 2)
    # config2 = rs.config()
    # config2.enable_device('932122060300')  # Replace with the serial number of your second camera
    # config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed

    # Start the pipelines
    pipeline1.start(config1)
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

    header = ['J1','J2','J3','J4','J5','J6', 'J7', 'Time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    # Set up the key listener
    print("Instructions Press 'O' to start recording data, 'P' to stop.")
    with keyboard.Listener(on_press=on_press) as listener:
        TICK_TIME = time.time()

        while True:
            if key_pressed == 'O':
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

            elif key_pressed == 'P':
                if is_writing:
                    print(f"Stopping data collection. File {trajectory_file} saved.")
                    trajectory_file_obj.close()  # Explicitly close the file when stopping
                    is_writing = False  # Stop writing data
                    trajectory_file = None

            # If we are writing data, perform the usual loop
            if is_writing:
                start_time = time.time()  # Record the start time of the loop iteration
                
                # Initialize the Image observation
                image_observation1 = None
                # Wait for a coherent set of frames: color frame
                frames1 = pipeline1.wait_for_frames()
                # Get the color frame
                color_frame1 = frames1.get_color_frame()
                # Convert the color frame to a numpy array
                image_observation1 = np.asanyarray(color_frame1.get_data())
                cv2.imshow('RealSense', image_observation1)

                # Initialize the Image observation
                # image_observation2 = None
                # Wait for a coherent set of frames: color frame
                # frames2 = pipeline2.wait_for_frames()
                # Get the color frame
                # color_frame2 = frames2.get_color_frame()
                # Convert the color frame to a numpy array
                # image_observation2 = np.asanyarray(color_frame2.get_data())
                # cv2.imshow('RealSense', image_observation2)


                # Wait for key events to keep the window responsive
                cv2.waitKey(1)

                # Get the current joint state
                action_observation = kb.get_current_joint_state()

                # Get the force torque data
                force_observations = ati_client.get_force_torque()
                rclpy.spin_until_future_complete(ati_client, force_observations)

                

                if action_observation is not None and force_observations.done() and image_observation1 is not None: # and image_observation2 is not None:
                    # Create the row for the CSV
                    TOCK_TIME = time.time() - TICK_TIME
                    response_kuka = [TOCK_TIME]
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
                    image_filename = os.path.join(image_folder, f"{row_index + 1}.png")  # Save as PNG, using the row index as filename
                    world_filename = os.path.join(world_folder, f"{row_index + 1}.png")  # Save as PNG, using the row index as filename
                    # print(f"Saving image as: {image_filename}")
                    # Save the image as a PNG file
                    # print(f"Saving image as: {image_filename}")
                    cv2.imwrite(image_filename, image_observation1)
                    # cv2.imwrite(world_filename, image_observation2)

                    row_index += 1  # Increment the index for the next image

                # Calculate the time taken for the current iteration
                elapsed_time = time.time() - start_time
                sleep_time = 0.033333 - elapsed_time  # Ensure the loop runs at approximately 0.03 sec intervals

                if sleep_time > 0:
                    time.sleep(sleep_time)

if __name__ == '__main__':
    main('no-sync/replay_traj_data', 1)


# import pyrealsense2 as rs
# import numpy as np
# import cv2

# ## serial number
# # ctx = rs.context()
# # devices = ctx.query_devices()

# # for dev in devices:
# #     print(dev.get_info(rs.camera_info.serial_number))

# # Create two pipelines for two cameras
# pipeline1 = rs.pipeline()
# # pipeline2 = rs.pipeline()

# # Configure the first pipeline (camera 1)
# config1 = rs.config()
# config1.enable_device('932122060300')  # Replace with the actual serial number of your first camera
# config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed

# # Configure the second pipeline (camera 2)
# # config2 = rs.config()
# # config2.enable_device('038522062901')  # Replace with the serial number of your second camera
# # config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed

# # Start the pipelines
# pipeline1.start(config1)
# # pipeline2.start(config2)

# try:
#     while True:
#         # Wait for frames from both cameras
#         frames1 = pipeline1.wait_for_frames()
#         # frames2 = pipeline2.wait_for_frames()

#         # Get color frames
#         color_frame1 = frames1.get_color_frame()
#         # color_frame2 = frames2.get_color_frame()

#         # Convert images to numpy arrays
#         color_image1 = np.asanyarray(color_frame1.get_data())
#         # color_image2 = np.asanyarray(color_frame2.get_data())

#         # Display the images
#         cv2.imshow('Camera world and robot', color_image1) #, color_image2)
#         # cv2.imshow('Camera 2', color_image2)

#         # Exit on pressing 'q'
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
# finally:
#     # Stop the pipelines when done
#     pipeline1.stop()
#     # pipeline2.stop()
#     cv2.destroyAllWindows()
