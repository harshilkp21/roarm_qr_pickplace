from setuptools import setup

package_name = 'roarm_control'

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
    maintainer='Harshil',
    maintainer_email='harshil@example.com',
    description='Pick and Place control node for RoArm M3 Pro',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_node = roarm_control.pick_place_node:main'
        ],
    },
)
