import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mag_heading'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='todo@example.com',
    description='Compute heading from calibrated magnetometer data',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mag_heading_node = mag_heading.mag_heading_node:main',
        ],
    },
)
