from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'obstacle_pkg'

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
    maintainer='vihaan',
    maintainer_email='vihaan30g@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sister = obstacle_pkg.obstacle_node:main',
            'sister2 = obstacle_pkg.obstacle_node_3d:main',
            'obstacle_pts = obstacle_pkg.obstacle_pts:main',
            'all_pts = obstacle_pkg.all_pts:main',
        ],
    },
)
