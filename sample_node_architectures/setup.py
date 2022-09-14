from setuptools import setup

package_name = 'sample_node_architectures'

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
    maintainer='pruvolo',
    maintainer_email='paullundyruvolo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_square = sample_node_architectures.drive_square:main',
            'get_odom_rpy = sample_node_architectures.get_odom_rpy:main'
        ],
    },
)
