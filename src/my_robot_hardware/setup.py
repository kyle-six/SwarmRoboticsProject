from setuptools import setup

package_name = 'my_robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'motor_controller', 'wifi_mesh_node', 'wifi_ibss_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hardware.launch.py']),
        ('share/' + package_name + '/launch', ['launch/wifi_mesh.launch.py']),
    ],
    install_requires=['setuptools', 'RPi.GPIO'],
    zip_safe=True,
    maintainer='kittputer',
    maintainer_email='kittputer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'motor_controller = motor_controller.motor_controller:main',
        'wifi_mesh_node = wifi_mesh_node.wifi_mesh_node:main',
        'wifi_ibss_node = wifi_ibss_node.wifi_ibss_node:main',
    ],
    },
)
