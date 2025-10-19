from setuptools import find_packages, setup

package_name = 'serial_comms'

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
    maintainer='xerix',
    maintainer_email='xerix@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = serial_comms.serial_node:main',
<<<<<<< HEAD
            'motorcontrol = serial_comms.motorcontrol:main'
=======
            'motorcontroller = serial_comms.motorcontroller:main',
>>>>>>> 29d4924361a9051a385446f9fa7bf3115587309e
        ],
    },
)
