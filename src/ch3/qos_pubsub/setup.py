from setuptools import find_packages, setup

package_name = 'qos_pubsub'

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
    maintainer='jcacace',
    maintainer_email='jonathan.cacace@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'publisher = qos_pubsub.qos_publisher:main',
        	'subscriber = qos_pubsub.qos_subscriber:main'
        ],
    },
)
