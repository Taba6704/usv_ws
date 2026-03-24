from setuptools import find_packages, setup

package_name = 'humidity_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mobsea',
    maintainer_email='mobsea@todo.todo',
    description='ROS 2 package for reading DHT11 temperature and humidity sensor',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'humidity_sensor_node = humidity_sensor.humidity_sensor_node:main',
            'lcd_display_node = humidity_sensor.lcd_display_node:main',
        ],
    },
)
