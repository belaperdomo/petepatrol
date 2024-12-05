from setuptools import find_packages, setup

package_name = 'pete_patrol'

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
    maintainer='volk',
    maintainer_email='volk@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_ps5controller = pete_patrol.ros2_ps5controller:main',
            'ros2_drive = pete_patrol.ros2_drive:main'
        ],
    },
)
