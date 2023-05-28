from setuptools import setup
import os
from glob import glob

package_name = 'tf2_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matttaylor',
    maintainer_email='mht3@illinois.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['static_turtle_tf2_broadcaster = tf2_demo.static_turtle_tf2_broadcaster:main',
                            'turtle_tf2_broadcaster = tf2_demo.turtle_tf2_broadcaster:main',
        ],
    },
)
