from setuptools import setup

package_name = 'robot_dispersion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/dispersion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='RSSI-based robot dispersion',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = robot_dispersion.robot_node:main',
            'rssi_simulator = robot_dispersion.rssi_simulator:main',
        ],
    },
)