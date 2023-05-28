from setuptools import setup
import os
from glob import glob

package_name = 'ctf_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'turtlebot3_description/urdf'), glob('turtlebot3_description/urdf/*.urdf')),
        (os.path.join('share', package_name, 'turtlebot3_description/meshes/sensors'), glob('turtlebot3_description/meshes/sensors/*')),
        (os.path.join('share', package_name, 'turtlebot3_description/meshes/bases'), glob('turtlebot3_description/meshes/bases/*')),
        (os.path.join('share', package_name, 'turtlebot3_description/meshes/wheels'), glob('turtlebot3_description/meshes/wheels/*')),
        (os.path.join('share', package_name, 'worlds/empty_worlds'), glob('worlds/empty_worlds/*')),
        (os.path.join('share', package_name, 'worlds/ctf_worlds'), glob('worlds/ctf_worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger/meshes'), glob('models/turtlebot3_burger/meshes/*')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger'), glob('models/turtlebot3_burger/*.sdf')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger'), glob('models/turtlebot3_burger/*.config')),
        (os.path.join('share', package_name, 'param'), glob('param/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matttaylor',
    maintainer_email='mht3@illinois.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtlebot = ctf_package.spawn_turtlebot:main',
            # 'tf_broadcaster = ctf_package.tf_broadcaster:main'
        ],
    },
)
