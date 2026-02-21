import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bluespark_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'calibration_files'), glob('calibration_files/*')),
        (os.path.join('share', package_name, 'ml_models'), glob('ml_models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Vision node for object detection, distance and angle estimation.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = bluespark_vision.vision_node:main',
        ],
    },
)
