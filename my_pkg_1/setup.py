from setuptools import find_packages, setup

package_name = 'my_pkg_1'

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
            'my_publisher = my_pkg_1.my_publisher:main',
            'my_subscriber = my_pkg_1.my_subscriber:main',
        ],
    },
)
