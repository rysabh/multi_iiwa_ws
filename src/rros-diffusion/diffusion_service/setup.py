from setuptools import find_packages, setup

package_name = 'diffusion_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'diffusion_policy_cam'],
    zip_safe=True,
    maintainer='cam',
    maintainer_email='rajtalan.rajtalan1998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diffusion_inference_service = diffusion_service.diffusion_inference_service:main',
            'diffusion_inference_client = diffusion_service.diffusion_inference_client:main',
        ],
    },
)
