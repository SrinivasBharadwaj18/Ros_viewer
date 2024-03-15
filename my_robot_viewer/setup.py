from setuptools import setup
import glob
import os

package_name = 'my_robot_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'), glob.glob(os.path.join('launch','*.launch.py'))),
        ('src/my_robot_viewer/config', glob.glob('mqtt_ws/src/my_robot_viewer/config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikhil',
    maintainer_email='srinivasbharadwaj18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sub_node=my_robot_viewer.subscriber:main",
            "mqtt_ros_bridge=my_robot_viewer.mqtt_ros_bridge:main"
        ],
    },
)
