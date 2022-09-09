from setuptools import setup

package_name = 'in_class_day03_solutions'

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
    maintainer='Paul Ruvolo',
    maintainer_email='paullundyruvolo@gmail.com',
    description='These are the sample solutions for the exercises described at https://comprobo22.github.io/in-class/day03',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_stop = in_class_day03_solutions.emergency_stop:main',
            'distance_emergency_stop = in_class_day03_solutions.distance_emergency_stop:main'
        ],
    },
)
