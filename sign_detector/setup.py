from setuptools import find_packages, setup
from glob import glob

package_name = 'sign_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', glob('templates/*')),
    ],
    install_requires=['setuptools'],

    maintainer='ali',
    maintainer_email='alisalimkhani2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = sign_detector.detector_node:main',
        ],
    },
)
