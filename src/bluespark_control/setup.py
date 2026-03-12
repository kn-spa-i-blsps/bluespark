from setuptools import setup
import os
from glob import glob

package_name = 'bluespark_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament/index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakub Balenkowski',
    maintainer_email='jakubbalenkowski@gmail.com',
    description='Package controlling BlueROV robot via RC Override',
    license='TODO: License decaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # when you type in 'ros2 run bluespark_control rc_override node
            # it would wake the main function from rc_override_node.py
            'rc_override_node = bluespark_control.rc_override_node:main',
            'vehicle_manager_node = bluespark_control.vehicle_manager_node:main'
        ],
    },
)