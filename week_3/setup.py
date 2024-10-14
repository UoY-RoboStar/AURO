import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'week_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pfr500',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='Solutions for week 3',
    license='EPL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_fsm = week_3.turtlebot3_fsm:main',
            'turtlebot3_fsm_rc = week_3.turtlebot3_fsm_rc:main',
            'turtlebot3_random_walk = week_3.turtlebot3_random_walk:main',
            'rviz_text_marker = week_3.rviz_text_marker:main'
        ],
    },
)
