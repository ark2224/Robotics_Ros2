from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cartesian_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*xml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dxy',
    maintainer_email='dongxiaoyang2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_control = cartesian_control.marker_control:main',
        ],
    },
)
