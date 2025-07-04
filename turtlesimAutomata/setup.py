from setuptools import find_packages, setup

package_name = 'turtlesimAutomata'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch', [
            'launch/turtlesim_automata.launch.py'
        ]),
    ],
    install_requires=['setuptools','rclpy', 'turtlesim', 'geometry_msgs'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'turtle_controller = turtlesimAutomata.turtle_controller:main',
        ],
    },
)
