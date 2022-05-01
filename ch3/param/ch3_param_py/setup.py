from glob import glob

import os

from setuptools import setup

package_name = 'ch3_param_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Homalozoa',
    maintainer_email='nx.tardis@gmail.com',
    description='Package for testing parameters in Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'soliloquist = ch3_param_py.soliloquist:main',
        ],
    },
)
