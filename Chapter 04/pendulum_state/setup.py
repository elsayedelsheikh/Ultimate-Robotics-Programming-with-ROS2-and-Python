from setuptools import find_packages, setup

package_name = 'pendulum_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/state_publisher_and_tf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jcacace',
    maintainer_email='jonathan.cacace@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pendulum_state = pendulum_state.state_publisher_and_tf:main'
        ],
    },
)
