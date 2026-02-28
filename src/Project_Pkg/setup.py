from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Project_Pkg'   # ROS recommends lowercase, but this works

def files(pattern):
    # glob returns [] if folder/file doesn't exist — safe for colcon
    return glob(pattern)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # ✅ removed the bad if-condition
    data_files=[
        # ament index + package.xml
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'), files('launch/*.py')),

        # maps
        (os.path.join('share', package_name, 'map'), files('map/*.yaml') + files('map/*.pgm')),

        # worlds
        (os.path.join('share', package_name, 'worlds'), files('worlds/*.world')),

        # configs / rviz
        (os.path.join('share', package_name, 'config'), files('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), files('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crri',
    maintainer_email='crri@todo.todo',
    description='Nav2 bringup with saved map and launches',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ✅ now correctly pointing to your Python node's main()
            'multi_nav_client = Project_Pkg.multi_nav_client:main',
        ],
    },
)

