from setuptools import setup

package_name = 'ch5_action_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='homalozoa',
    maintainer_email='nx.tardis@gmail.com',
    description='ROS 2 action demo wroten in Python.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'self_action_py = ch5_action_py.self_node:main',
        ],
    },
)
