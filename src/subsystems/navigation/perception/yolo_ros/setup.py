from setuptools import find_packages, setup

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'yolo', 'ultralytics'],
    zip_safe=True,
    maintainer='umdloop',
    maintainer_email='umdloop@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = yolo_ros.my_node:main',
            'yolo_ros = yolo_ros.image_publisher:main',
            'inference = yolo_ros.img_inference:main',
        ],
    },
)
