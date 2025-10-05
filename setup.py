from setuptools import setup

package_name = 'robot_behaviour'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],   # must match folder name exactly
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch.py']),
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
        ],
    },
)
