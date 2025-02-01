from setuptools import setup

package_name = 'auto_quad_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='mohamadd.abdelrahimWgmail.com',
    description='A package to scale LiDAR data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scale_lidar_node = auto_quad_description.lidar_node:main',
        ],
    },
)