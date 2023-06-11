from setuptools import setup

package_name = 'zumo_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='van',
    maintainer_email='van@todo.todo',
    description='Zumo Line Follower ROS2 Package.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = zumo_line_follower.line_follower_wireless:main',
            'stop_service = zumo_line_follower.stop_service:main'
        ],
    },
)
