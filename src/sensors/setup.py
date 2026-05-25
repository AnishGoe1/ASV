from setuptools import find_packages, setup

package_name = 'sensors'

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
    description='Package to control different sensors of the ASV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "batt_pub = sensors.battery_pub:main"
        ],
    },
)
