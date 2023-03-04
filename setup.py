from setuptools import setup

package_name = 'move_turtle'

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
    maintainer='andre',
    maintainer_email='andre@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtle = move_turtle.move_turtle:main',
            'odometry = move_turtle.odometry:main',
            'move_with_odometry = move_turtle.move_with_odometry:main',
            'rotate_with_odometry = move_turtle.rotate_with_odometry:main',
            'rotate_with_odometry_with_initial_position = move_turtle.rotate_with_odometry_with_initial_position:main',

        ],
    },
)
