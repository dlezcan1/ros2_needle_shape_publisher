from setuptools import setup
import glob

package_name = 'needle_shape_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*.launch.*')),
        ('share/' + package_name + "/needle_data", glob.glob('needle_data/*/needle_params*.json')),
        # ('share/' + package_name + "/config", glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dimitri Lezcano',
    maintainer_email='dlezcan1@jhu.edu',
    description='Needle Shape Sensing Python ROS 2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'sensorized_shapesensing_needle = needle_shape_publisher.sensorized_shape_sensing_needle:main',
                'sensorized_shapesensing_needle_threaded = needle_shape_publisher.sensorized_shape_sensing_needle_threaded:main',
                'sensorized_needle = needle_shape_publisher.sensorized_needle:main',
                'shapesensing_needle = needle_shape_publisher.shape_sensing_needle:main'
        ],
    },
)
