from setuptools import setup
from glob import glob
import os


package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angel',
    maintainer_email='angeloob11@gmail.com',
    description='Simple object detection',
    license='Apache licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['obstacle_detector = object_detection.obstacle_detector:main',
                            'obstacle_monitor = object_detection.obstacle_monitor:main',
                            'obstacle_detector_2 = object_detection.obstacle_detector_2:main'
        ],
    },
)
