from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'GenericHRLoader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aurum',
    maintainer_email='aurum@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'HR_Joint_Pub = GenericHRLoader.HR_Joint_Pub:main',
            'HR_PID_Joint_Controller = GenericHRLoader.HR_PID_Joint_Controller:main'
        ],
    },
)
