from setuptools import find_packages, setup

package_name = 'cmd_vel_mux'

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
    maintainer_email='BASSOT@my.erau.edu',
    description='Command velocity multiplexer for RC/autonomous mode selection',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_mux_node = cmd_vel_mux.cmd_vel_mux_node:main',
        ],
    },
)
