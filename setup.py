from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_controller'

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
    maintainer='eduardohufg',
    maintainer_email='eduardochavezmartin10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = turtle_controller.turtle_move:main',
            'path_generator = turtle_controller.path_generator:main',
            'tracker_points = turtle_controller.tracker_points:main',
            'close_loop = turtle_controller.close_loop:main',
            'odometry = turtle_controller.odometry:main',
            'pz_close = turtle_controller.close_loop_pz:main',
        ],
    },
)
