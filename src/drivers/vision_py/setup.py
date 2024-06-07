from setuptools import find_packages, setup
import os

package_name = 'vision_py'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/custom_model_lite', package_files('custom_model_lite')),
        ('share/' + package_name + '/custom_model_lite/saved_models', package_files('custom_model_lite/saved_models')),
        ('share/' + package_name + '/custom_model_lite/saved_models/assets', package_files('custom_model_lite/saved_models/assets')),
        ('share/' + package_name + '/custom_model_lite/saved_models/variables', package_files('custom_model_lite/saved_models/variables'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owhenthesaints',
    maintainer_email='owen.simon@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = vision_py.vision_py:main'
        ],
    },
)
