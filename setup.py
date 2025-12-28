import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'minicar_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # robot navigation files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # robot simulation environment: road_env
        (os.path.join('share', package_name, 'minicar_simulation/worlds'), 
            glob('minicar_simulation/worlds/*.world')
        ),
        (os.path.join('share', package_name, 'minicar_simulation/models/road_env'),
            ['minicar_simulation/models/road_env/model.config',
             'minicar_simulation/models/road_env/model.sdf']
        ),
        (os.path.join('share', package_name, 'minicar_simulation/models/road_env/meshes'),
            glob('minicar_simulation/models/road_env/meshes/*')
        ),

        # robot description files
        (os.path.join('share', package_name, 'description/launch'), glob('description/launch/*.py')),
        (os.path.join('share', package_name, 'description/config'), glob('description/config/*.yaml')),
        (os.path.join('share', package_name, 'description/urdf'), glob('description/urdf/*.urdf')),
        (os.path.join('share', package_name, 'description/urdf'), glob('description/urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='peisuke.com@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'local_nav = minicar_navigation.local_nav_node:main',
        ],
    },
)
