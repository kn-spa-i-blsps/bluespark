from setuptools import setup
import os
from glob import glob

package_name = 'bluespark_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # THIS line is what makes `ros2 launch bluespark_bringup <x>.launch.py`
        # work: it installs every *.launch.py from the launch/ folder into the
        # package's share/ directory, where ros2 launch looks for them.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jappa',
    maintainer_email='jappa414@gmail.com',
    description='Launch files that bring up the BlueSpark system per Raspberry Pi role.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ./bs: session timeline (started by governer.launch.py log_dir:=...)
            'timeline_recorder = bluespark_bringup.timeline_recorder:main',
            # ./bs mission <name>: one mission with logs + safety stop
            'mission_runner = bluespark_bringup.mission_runner:main',
            # ./bs arm / disarm / mode / pwm / neutral / status / althold / test
            'vehicle = bluespark_bringup.vehicle_cli:main',
        ],
    },
)
