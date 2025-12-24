from setuptools import find_packages, setup

package_name = 'hydrakon_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya S',
    maintainer_email='as2397@hw.ac.uk',
    description='Folder for managing anything related to the vehicle in Carla',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_spawner = hydrakon_manager.vehicle_node:main',
            'manual_control = hydrakon_manager.manual_control:main',
            'carla_bridge = hydrakon_manager.carla_to_ros_bridge:main',
            'ins_node = hydrakon_manager.ins_node:main',
        ],
    },
)
