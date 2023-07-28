from setuptools import setup
from glob import glob
import os

package_name = 'bumpgo'

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
    description='Bump and go pkg',
    license='Apache licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['bump_go = bumpgo.bump_go:main',
                            'bump_go_2 = bumpgo.bump_go_2:main',
                            'bump_go_3 = bumpgo.bump_go_3:main'
        ],
    },
)
