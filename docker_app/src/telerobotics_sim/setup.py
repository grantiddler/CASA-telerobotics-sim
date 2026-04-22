from setuptools import find_packages, setup

package_name = 'telerobotics_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/rover_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='grantdridde7@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mujoco = telerobotics_sim.mujoco_node:main',
            'listener = telerobotics_sim.position_listener:main',
            'control = telerobotics_sim.control_publisher:main',
            'rover_velocity_controller = telerobotics_sim.rover_velocity_controller:main',
        ],
    },
)
