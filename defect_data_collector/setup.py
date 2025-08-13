from setuptools import setup

package_name = 'defect_data_collector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/collector.yaml']),
        ('share/' + package_name + '/launch', ['launch/defect_collection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    description='PoseArray simulator + low-latency collector to Parquet/CSV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'collector = defect_data_collector.collector_node:main',
            'simulator = defect_data_collector.simulator_node:main',
        ],
    },
)
