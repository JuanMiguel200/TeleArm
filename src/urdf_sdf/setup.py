from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'urdf_sdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # Asegura que los archivos launch se instalen
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),  # Incluye los URDF
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanmi',
    maintainer_email='juanmi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
