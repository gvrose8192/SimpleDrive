from setuptools import find_packages, setup

package_name = 'simple_drive'

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
    maintainer='ubuntu',
    maintainer_email='gvrose8192@gmail.com',
    description='Simple Drive - go forward 50 cm and then back 50 cm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_drive_node = simple_drive.simple_drive_node:main',
            'reset_service_node = simple_drive.reset_service_node:main',
        ],
    },
)
