from setuptools import find_packages, setup

package_name = 'multi_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roman Sinkus',
    maintainer_email='sinkusroman@gmail.com',
    description='Reusable control API (ArmFleet / Arm) for multi-UR-arm MoveIt packages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
