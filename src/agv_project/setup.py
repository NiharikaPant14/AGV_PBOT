import os
from glob import glob
from setuptools import setup 

package_name = 'agv_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install all .launch.py files from the 'launch' folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        
        # Install all .yaml files from the 'config' folder
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        # Install all .sdf/.world files from the 'worlds' folder
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
        
        # Install all map files from the 'maps' folder
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alekhya',
    maintainer_email='alekhya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_sender_node = agv_project.goal_sender_node:main', 
            'commander = agv_project.goal_sender_node:main'  
        ],
    },
)
