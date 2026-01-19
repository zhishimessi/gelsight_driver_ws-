from setuptools import setup
import os
from glob import glob

package_name = 'markermotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'scikit-image',
        'numpy',
        'opencv-python',
        'scipy',
        'matplotlib'
    ],
    zip_safe=True,
    maintainer='donghy',
    maintainer_email='donghy2022@shanghaitech.edu.cn',
    description='GelSight sensor bridge for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gelsight_driver = markermotion.gelsight_bridge_basic:main',
        ],
    },
)
