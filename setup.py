<<<<<<< HEAD
from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'self_localization'
=======
from setuptools import setup

package_name = 'aruco_multi_localization'
>>>>>>> ef5075f81dde4c8a22c1986613210ef31dad6eb6

setup(
    name=package_name,
    version='0.0.0',
<<<<<<< HEAD
    packages=find_packages(exclude=['test']),
=======
    packages=[
        package_name,
        package_name + '.aruco_multi_localizer'
    ],
>>>>>>> ef5075f81dde4c8a22c1986613210ef31dad6eb6
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< HEAD
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
=======
        ('share/' + package_name, ['camera_calibration_params.npz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumit',
    maintainer_email='sumit.mor@hof-university.de',
    description='Multi ArUco marker localization in ROS2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'multi_aruco_localizer = aruco_multi_localization.aruco_multi_localizer.multi_aruco_localizer:main'
>>>>>>> ef5075f81dde4c8a22c1986613210ef31dad6eb6
        ],
    },
)
