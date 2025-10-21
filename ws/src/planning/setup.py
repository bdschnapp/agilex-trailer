from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include service files
        (os.path.join('share', package_name, 'srv'), 
         glob('srv/*.srv')),
        # Include configuration files if any
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy', 
        'matplotlib',
        'shapely',
        'numba',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ben.schnapp@cplaneai.com',
    description='Integrated truck-trailer planning pipeline with forward and reverse motion capabilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_pcd_publisher = planning.static_pcd_publisher:main',
            'pcd_to_occupancy_grid = planning.pcd_to_occupancy_grid:main',
        ],
    },
)
