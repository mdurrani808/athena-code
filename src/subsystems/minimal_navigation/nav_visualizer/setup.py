from setuptools import find_packages, setup

package_name = 'nav_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'matplotlib', 'numpy'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='mdurrani808@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_visualizer_node = nav_visualizer.nav_visualizer_node:main'
        ],
    },
)
