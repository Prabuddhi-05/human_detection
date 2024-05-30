from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'human_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prabuddhi',
    maintainer_email='prabuddhi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'fisheye_camera = human_detection.fisheye_camera:main',
          'detection_fisheye = human_detection.detection_fisheye:main',
          'detection_rgbd = human_detection.detection_rgbd:main',
          'fisheye_camera_multi = human_detection.fisheye_camera_multi:main',
          'fisheye_camera_multiple = human_detection.fisheye_camera_multiple:main',
        ],
    },
)