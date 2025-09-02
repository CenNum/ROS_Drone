from setuptools import find_packages, setup

package_name = 'drone_utilities'

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
    maintainer='orangepi',
    maintainer_email='2874dsl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_pose_converter = drone_utilities.odom_to_pose_converter:main',
            't265_to_mav = drone_utilities.t265_to_mav:main',
            'waypoint_land_navigator = drone_utilities.waypoint_land_navigator:main'
        ],
    },
)
