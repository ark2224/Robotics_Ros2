from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'assignment4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = assignment4.robot:main',
            'gui = assignment4.gui:main',
            'estimator = assignment4.est:main',
            'est_gt = assignment4.gui_plugin:main',
            'autograde = assignment4.AutoGrade:main', 
        ],
    },
)
