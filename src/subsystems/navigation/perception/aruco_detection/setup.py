from setuptools import find_packages, setup

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv-bridge', 'rclpy'],
    zip_safe=True,
    maintainer='abhinavkota06',
    maintainer_email='abhinav.kota06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aruco_node = aruco_detection.aruco_node:main",
            "correction_node = aruco_detection.correction_node:main",
            "dummy_BB_node = aruco_detection.dummy_BB_node:main"
        ],
    },
)
