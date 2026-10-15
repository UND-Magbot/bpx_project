from glob import glob

from setuptools import setup

package_name = 'bpx_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='und',
    description='MirrorMe BPX quadruped ROS 2 bringup',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bpx_bridge = bpx_ros.bridge:main',
            'map_to_odom = bpx_ros.map_to_odom:main',
        ],
    },
)
