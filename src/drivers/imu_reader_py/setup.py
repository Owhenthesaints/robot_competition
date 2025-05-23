from setuptools import find_packages, setup

package_name = 'imu_reader_py'

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
    maintainer='owhenthesaints',
    maintainer_email='owen.simon@epfl.ch',
    description='Minimal pub',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'imu_publisher = imu_reader_py.imu_publisher:main',
        ],
    },
)
