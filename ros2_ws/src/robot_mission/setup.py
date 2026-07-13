from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_mission'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EasonSong208',
    maintainer_email='easonsong208@users.noreply.github.com',
    description='Read-only mission readiness checks for the Hiwonder JetRover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_smoke_test = robot_mission.motion_smoke_test:main',
            'out_and_back_test = robot_mission.out_and_back_test:main',
            'preflight = robot_mission.preflight:main',
            'turn_step_test = robot_mission.turn_step_test:main',
        ],
    },
)
