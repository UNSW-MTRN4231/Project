import os
from glob import glob
from setuptools import setup

package_name = 'bottle_flip'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='nazar',
    author_email='dilnazardolkun@outlook.com',
    maintainer='nazar',
    maintainer_email='dilnazardolkun@outlook.com',
    description='Description of the package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bottle_flip = bottle_flip.ur_driver_flip:main',
        ],
        
    },

    
)
