from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'garuda_description'

# Helper function to recursively find all files in a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # Handle model files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro') + glob('urdf/*.gazebo')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prudhviraj',
    maintainer_email='prudhvirajchalapaka@gmail.com',
    description='This Package deals with garuda robot description and ROS2_GZ_BRIDGE',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)