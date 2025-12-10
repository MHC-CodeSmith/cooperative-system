from setuptools import setup

package_name = 'aruco_multi_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + '.aruco_multi_localizer'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
