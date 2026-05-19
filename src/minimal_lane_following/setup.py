from glob import glob
from setuptools import find_packages, setup

package_name = 'minimal_lane_following'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuele Moranzoni',
    maintainer_email='samuele@example.com',
    description='Minimal camera-based lane perception and control for RoboMaster.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_perception_node = minimal_lane_following.lane_perception_node:main',
            'lane_controller_node = minimal_lane_following.lane_controller_node:main',
        ],
    },
)
