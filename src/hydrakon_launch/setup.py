from setuptools import find_packages, setup

package_name = 'hydrakon_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/custom.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya S',
    maintainer_email='as2397@hw.ac.uk',
    description='Singluar Launch file for launching all nodes for HydrakonSimV2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
