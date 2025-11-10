# src/cv/setup.py

from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'cv'

# Find packages in the 'src' directory
packages = find_packages(where='src')

# Collect all files from the models directory recursively
model_data_files = []
for (path, directories, filenames) in os.walk('models'):
    for filename in filenames:
        source_path = os.path.join(path, filename)
        install_path = os.path.join('share', package_name, path)
        model_data_files.append((install_path, [source_path]))

setup(
    name=package_name,
    version='0.0.1',
    packages=packages,
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name, ['package.xml']),
    ] + model_data_files,  # Correctly concatenate the lists
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='alisalimkhani2004@gmail.com',
    description='Computer vision package for traffic sign control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = cv.controller_node:main',
        ],
    },
)