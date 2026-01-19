import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hanoi_tower'

setup(
    name=package_name,
    version='1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models', 'bin'), 
            glob('models/bin/*')),
        (os.path.join('share', package_name, 'models', 'cube_small'), 
            [f for f in glob('models/cube_small/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'cube_small', 'materials', 'scripts'), 
            glob('models/cube_small/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'cube_small', 'materials', 'textures'), 
            glob('models/cube_small/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'cube_medium'), 
            [f for f in glob('models/cube_medium/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'cube_medium', 'materials', 'scripts'), 
            glob('models/cube_medium/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'cube_medium', 'materials', 'textures'), 
            glob('models/cube_medium/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'cube_large'), 
            [f for f in glob('models/cube_large/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'cube_large', 'materials', 'scripts'), 
            glob('models/cube_large/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'cube_large', 'materials', 'textures'), 
            glob('models/cube_large/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'camera'), 
            glob('models/camera/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boron',
    maintainer_email='shankhoghosh123@gmail.com',
    description='Tower of Hanoi using xArm6 robot',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'vision_node = hanoi_tower.vision_node:main',
            'hanoi_solver = hanoi_tower.hanoi_solver:main',
            'motion_executor = hanoi_tower.motion_executor:main',
        ],
    },
)
