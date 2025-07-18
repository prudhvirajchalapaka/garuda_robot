from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'garuda_navigation'

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
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prudhviraj',
    maintainer_email='prudhvirajchalapaka@gmail.com',
    description='This package is developed to give the garuda robot autonomous navigation functionality',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
