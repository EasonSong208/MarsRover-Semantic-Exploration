from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'semantic_perception'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EasonSong208',
    maintainer_email='easonsong208@users.noreply.github.com',
    description='Fake and PIDNet-S hazard5 semantic perception interfaces.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_semantic_node = '
            'semantic_perception.semantic_perception_node:main',
            'pidnet_semantic_node = '
            'semantic_perception.pidnet_semantic_node:main',
            'pidnet_static_test = '
            'semantic_perception.pidnet_static_test:main',
            'semantic_fusion_compat = '
            'semantic_perception.semantic_fusion_compat_node:main',
        ],
    },
)
