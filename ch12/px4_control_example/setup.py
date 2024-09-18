from setuptools import find_packages, setup

package_name = 'px4_control_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/x500_gazebo_sensors.launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_motion_control = px4_control_example.px4_motion_control:main',
            'quadcopter_gazebo_integration = px4_control_example.quadcopter_gazebo_integration:main'
        ],
    },
)
