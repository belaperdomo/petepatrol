from setuptools import find_packages, setup

package_name = 'person_tracker'

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
    maintainer='msakal2024',
    maintainer_email='msakal2024@my.fit.edu',
    description='ROS 2 package for person tracking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_tracker_node = person_tracker.person_tracker_node:main'
        ],
    },
)
