from setuptools import setup

package_name = 'robot_behaviour'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Simple robot behaviour package (obstacle avoidance + FSM)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = robot_behaviour.obstacle_avoidance:main',
            'fsm_controller = robot_behaviour.fsm_controller:main',
            'teleop_simple = robot_behaviour.teleop_simple:main'
        ],
    },
)
