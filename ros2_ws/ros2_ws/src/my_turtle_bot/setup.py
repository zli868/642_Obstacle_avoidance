from setuptools import setup

package_name = 'my_turtle_bot'

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
    maintainer='zeren li',
    maintainer_email='zli06211@umd.edu',
    description='Obstacle avoidance package for TurtleBot3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
            'avoidance = my_turtle_bot.avoidance:main',
            'my_avoidance = my_turtle_bot.my_avoidance:main'
            ],

        },
)
