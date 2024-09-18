from setuptools import find_packages, setup

package_name = 'bt_surveillance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/surveillance_system.launch.py']),
        ('share/' + package_name, ['config/surveillance_system.yaml']),
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
            'sample_action = bt_surveillance.sample_action:main',
            'surveillance_system = bt_surveillance.survellinace_system:main',
        ],
    },
)
