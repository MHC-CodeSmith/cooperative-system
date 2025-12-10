from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'self_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmichael2',
    maintainer_email='johannes.michael@hof-university.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_self_localization = self_localization.aruco_self_localization:main',
            'image_publisher = self_localization.image_publisher:main',
            'multi_aruco_localization = self_localization.multi_aruco_localization:main'
        ],
    },
)
