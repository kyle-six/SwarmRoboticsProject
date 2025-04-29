from setuptools import setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, "cmd_vel_publisher"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/my_robot.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/my_robot.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Robot description package with Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'cmd_vel_publisher = cmd_vel_publisher.cmd_vel_publisher:main'
    ],
},
)
