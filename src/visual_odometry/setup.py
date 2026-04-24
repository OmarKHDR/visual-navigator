from setuptools import find_packages, setup
import os

package_name = 'visual_odometry'

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
    maintainer='omark',
    maintainer_email='omarsliman37@gmail.com',
    description='Visual Odometry node for Navigation System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry = visual_odometry.visual_odometry:main'
        ],
    },
)
