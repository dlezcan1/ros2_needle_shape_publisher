from setuptools import setup

package_name = 'needle_shape_publisher'

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
    maintainer='Dimitri Lezcano',
    maintainer_email='dlezcan1@jhu.edu',
    description='Needle Shape Sensing Python node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'sensorized_shapesensing_needle = needle_shape_publisher.sensorized_shape_sensing_needle:main'
        ],
    },
)