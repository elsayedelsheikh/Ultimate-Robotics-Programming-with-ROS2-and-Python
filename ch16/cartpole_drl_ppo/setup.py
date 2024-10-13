from setuptools import find_packages, setup

package_name = 'cartpole_drl_ppo'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartpole_training = cartpole_drl_ppo.cartpole_training:main',
            'cartpole_prediction = cartpole_drl_ppo.cartpole_prediction:main',
            'plot_data = cartpole_drl_ppo.plot_data:main'
        ],
    },
)
