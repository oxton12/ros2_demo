from setuptools import find_packages, setup

package_name = 'ros2_demo'

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
    maintainer='alexey-dev',
    maintainer_email='alexey.obshatko@compvisionsys.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['position_estimator = ros2_demo.nodes.position_estimator:main',
                            'yolo_vision = ros2_demo.nodes.yolo_vision:main',
                            'camera_driver = ros2_demo.nodes.camera_driver:main',
                            'serial_writer = ros2_demo.nodes.serial_writer:main'
        ],
    },
    scripts=[
      'stream_server/start_docker.sh'
    ]
)
