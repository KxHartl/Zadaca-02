from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'zadaca_06'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'kuka/visual'), glob('*kuka/visual/*')),
        (os.path.join('share', package_name, 'kuka/collision'), glob('*kuka/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kresimirhartl',
    maintainer_email='kresimirhartl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = zadaca_06.state_publisher:main'
        ],
    },
)
