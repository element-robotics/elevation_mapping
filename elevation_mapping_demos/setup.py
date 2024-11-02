from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elevation_mapping_demos'\

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch*'))),
    (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
]

# Recursively add all files in the config directory, preserving the structure
for root, dirs, files in os.walk('config'):
    install_path = os.path.join('share', package_name, root)  # Maintain the directory structure
    file_paths = [os.path.join(root, f) for f in files]
    data_files.append((install_path, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tstreet',
    maintainer_email='taaj.street@elementrobotics.space',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'tf_to_pose_publisher = elevation_mapping_demos.tf_to_pose_publisher:main',  
        ],
    },
)
