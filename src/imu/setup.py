from setuptools import find_packages, setup

package_name = 'imu'

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
    description='Package to get data from the IMU sensor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "imu_publisher =imu.imu_publisher:main",
            "imu_logger = imu.imu_logger:main"

        ],
    },
)
