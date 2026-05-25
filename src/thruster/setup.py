from setuptools import find_packages, setup

package_name = 'thruster'

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
    maintainer='anish',
    maintainer_email='anishgoel1129@gmail.com',
    description='Package to control the ASV via the keyboard',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thruster_teleop = thruster.thrust_teleop_publisher:main',
            'thruster_sub = thruster.thrust_subscriber:main',
        ],
    },
)
