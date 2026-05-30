from setuptools import find_packages, setup

package_name = 'bluespark_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    # Automatically find all packages in the directory, ignoring the test folder
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jappa',
    maintainer_email='jappa414@gmail.com',
    description='Autonomy based on py_trees framework',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonomy_node = bluespark_autonomy.autonomy_node:main',
        ],
    },
)