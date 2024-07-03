from setuptools import find_packages, setup

package_name = 'moveit_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cam',
    maintainer_email='cam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher_test = moveit_motion.joint_state_publisher_test:main',
            'motion_planning = moveit_motion.motion_planning:main',
            'mhubli_motion = moveit_motion.mhubli_motion:main',
            'mhubli_motion_exec_joint_traj = moveit_motion.mhubli_motion_exec_joint_traj:main',
            'mhubli_motion_follow_joint_traj = moveit_motion.mhubli_motion_follow_joint_traj:main',
            'mhubli_motion_follow_joint_traj_current_robot_position = moveit_motion.mhubli_motion_follow_joint_traj_current_robot_position:main',
            'test_joint_state_subscriber = moveit_motion.test_joint_state_subscriber:main',
            'test = moveit_motion.test:main',
            'execute_on_real_robot = moveit_motion.execute_on_real_robot:main',
            'raj_motion_plan = moveit_motion.raj_motion_plan:main',
        ],
    },
)
