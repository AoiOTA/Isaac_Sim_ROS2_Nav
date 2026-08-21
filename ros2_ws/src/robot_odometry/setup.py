from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AoiOTA',
    maintainer_email='liang_yibo@hdu.edu.cn',
    description='Wheel odometry and bounded raw-to-corrected IMU calibration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_yaw_calibrator = '
            'robot_odometry.imu_calibration_node:main',
            'wheel_odometry_node = robot_odometry.wheel_odometry_node:main',
        ],
    },
)
