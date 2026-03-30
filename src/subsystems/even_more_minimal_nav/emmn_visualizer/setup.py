from setuptools import find_packages, setup

package_name = 'emmn_visualizer'

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
    description='Debug visualizer for even_more_minimal_nav',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emmn_visualizer_node = emmn_visualizer.emmn_visualizer_node:main',
        ],
    },
)
