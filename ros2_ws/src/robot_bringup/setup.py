from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_bringup'

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
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='AoiOTA',
    maintainer_email='liang_yibo@hdu.edu.cn',
    description='Validated ROS-side mode orchestration for Isaac Sim navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ideal_localization_tf = '
            'robot_bringup.ideal_localization_tf:main',
            'initial_pose_policy = robot_bringup.initial_pose_policy:main',
            'map_manifest = robot_bringup.map_manifest:main',
            'nav2_activation_gate = robot_bringup.activation_gate:main',
            'ordered_shutdown = robot_bringup.ordered_shutdown:main',
        ],
    },
)
