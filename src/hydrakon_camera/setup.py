from setuptools import find_packages, setup

package_name = 'hydrakon_camera'

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
    description='Singular Folder for handling all camera related tasks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_camera_spawner = hydrakon_camera.depth_camera:main',
            'rgb_camera_spawner = hydrakon_camera.camera:main',
            'depth_anything_processor = hydrakon_camera.depth_anything:main',
            'cone_detector = hydrakon_camera.YOLO:main',
            'cone_locator = hydrakon_camera.cone_locator:main',
        ],
    },
)
