from setuptools import find_packages, setup

package_name = 'assignment1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test','pyarmor_runtime_000000']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ark2224@columbia.edu',
    description='assignment1 from Drew Kalasky',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solution = assignment1.solution:main',
            'marker_publisher = assignment1.marker_publisher:main',
            'autograde = assignment1.autograde:main',
        ],
    },
)
