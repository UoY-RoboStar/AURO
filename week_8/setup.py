# Based on https://answers.ros.org/question/397319/how-to-copy-folders-with-subfolders-to-package-installation-path/

import os
from setuptools import find_packages, setup

package_name = 'week_8'

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['models/', 'launch/', 'worlds/', 'rviz/', 'urdf/', 'maps/', 'params/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pfr500',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = week_8.robot_controller:main',
            'joint_state_republisher = week_8.joint_state_republisher:main',
            'path_publisher = week_8.path_publisher:main',
            'simple_commander = week_8.simple_commander:main',
            'autonomous_navigation = week_8.autonomous_navigation:main',
            'autonomous_navigation_multithreaded = week_8.autonomous_navigation_multithreaded:main',
        ],
    },
)
