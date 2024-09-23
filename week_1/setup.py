from setuptools import find_packages, setup

package_name = 'week_1'

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
    maintainer='pfr500',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='Sample solution for week 1',
    license='EPL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_listener = week_1.turtle_listener:main',
            'turtle_controller = week_1.turtle_controller:main'
        ],
    },
)
