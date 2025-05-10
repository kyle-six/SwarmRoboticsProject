from setuptools import setup

package_name = 'my_swarm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, "wifi_neighbors_aggregator"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wifi_aggregator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kittputer',
    maintainer_email='kittputer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_controller = my_swarm_control.swarm_controller:main',
            'wifi_aggregator = wifi_neighbors_aggregator.wifi_neighbors_aggregator:main',
            'fake_robot_simulator = my_swarm_control.fake_robot_simulator:main',
        ],
    },
)
