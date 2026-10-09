from setuptools import find_packages, setup

package_name = 'semih_perception'

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
    maintainer='jangjunseo',
    maintainer_email='jangjunseo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detection_node = semih_perception.yolo_detection_node:main',
            'target_3d_node = semih_perception.target_3d_node:main',
            'arm_reach_node = semih_perception.arm_reach_node:main',
        ],
    },
)
