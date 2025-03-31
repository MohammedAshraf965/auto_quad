from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auto_quad_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammed-ashraf',
    maintainer_email='mohamadd.abdelrahim@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['stereo_feedback = auto_quad_localization.stereo_feedback:main',
                            'camera_info_publisher = auto_quad_localization.camera_info_publisher:main',
                            'camera_calibration_service = auto_quad_localization.camera_calibration_service:main',
                            'feature_matcher = auto_quad_localization.feature_matcher:main',
                            'visual_odometry = auto_quad_localization.visual_odometry:main',
        ],
    },
)
