from setuptools import find_packages, setup

package_name = 'python_sequence_planner'

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
    maintainer='battery',
    maintainer_email='samrudhmoode@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_sequence_planner = python_sequence_planner.python_sequence_planner:main',
            'dual_sequence_planner_test = python_sequence_planner.dual_sequence_planner_test:main',
            'find_fk = python_sequence_planner.find_fk:main',
        ],
    },
)
