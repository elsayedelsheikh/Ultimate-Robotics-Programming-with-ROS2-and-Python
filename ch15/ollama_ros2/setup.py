from setuptools import find_packages, setup

package_name = 'ollama_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/navigation.launch.py']),
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
            'ollama_interface = ollama_ros2.ollama_interface:main',
            'ollama_tools = ollama_ros2.ollama_tools:main',
            'llm_mobile = ollama_ros2.llm_mobile_robotics:main',

        ],
    },
)
