from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'calibration': ['fast_poisson.py'],  
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
    description='GelSight calibration package for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_realtime = calibration.depth_realtime:main',
        ],
    },
)
