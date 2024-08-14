# Motion Sequence Planner

This package contains the `motion_sequence_planner` ROS 2 package, which is designed to assist in planning and executing motion sequences for robotic systems.

## Folder Structure

```
motion_sequence_planner/
│
├── src/
│   └── motion_seq_planner.cpp
│
├── CMakeLists.txt
│
├── package.xml
│
└── README.md
```

## Package Creation

To create this package, use the following command:

```bash
ros2 pkg create motion_sequence_planner --build-type ament_cmake
```

## Modifications

The following modifications are required for this package to function properly with your specific setup:

1. **CMakeLists.txt & package.xml**
    - All necessary changes have been commented within the files. You can replicate these changes according to your system's needs.

2. **Source Code Adjustments**
    - **Add Suitable Imports:** Ensure that all necessary ROS 2 and MoveIt2 libraries are correctly imported in `motion_seq_planner.cpp`.
    - **Move Group Names:** Update the move group names in the code to match the configuration of your robot.
    - **Base Link and End Effector Link:** Modify the names of the base link and end effector link to correspond with your robotic setup.

## Usage

After making the required changes, build the package using the following commands:

```bash
colcon build --symlink-install
```

Then, you can run the node as needed for your application.