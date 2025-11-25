from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_hello'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rayan',
    maintainer_email='rayan@example.com',
    description='Hello World Gazebo example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pan_tilt_motion = gazebo_hello.pan_tilt_motion:main',
            'face_detector = gazebo_hello.face_detector:main',
            'face_tracker = gazebo_hello.face_tracker:main',
            'image_viewer = gazebo_hello.image_viewer:main',
            'target_teleop = gazebo_hello.target_teleop:main',

        ],
    },
)

