from setuptools import setup

package_name = 'ch2_node_py'

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
    description='Python node demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node2go=ch2_node_py.node2go:main',
            'node2gons=ch2_node_py.node2gons:main'
        ],
    },
)
