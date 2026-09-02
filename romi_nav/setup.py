import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'romi_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Faysal Shah',
    maintainer_email='shahfaysal6969@gmail.com',
    description='ROMI Lab navigation harness: sim bringup, SLAM, Nav2, and the recommendation-vs-execution logging node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recommendation_node = romi_nav.recommendation_node:main',
        ],
    },
)
