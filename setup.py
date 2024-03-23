from setuptools import find_packages, setup

package_name = 'zadaca_04'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kresimirhartl',
    maintainer_email='kresimirhartl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "brojcanik = zadaca_04.brojcanik:main",
            "kvadriranje_broja = zadaca_04.kvadriranje_broja:main"
        ],
    },
)
