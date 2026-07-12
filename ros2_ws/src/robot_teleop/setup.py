from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AoiOTA',
    maintainer_email='liang_yibo@hdu.edu.cn',
    description='Deadman-protected W/A/S/D keyboard control for mapping.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = robot_teleop.keyboard_teleop:main',
        ],
    },
)
