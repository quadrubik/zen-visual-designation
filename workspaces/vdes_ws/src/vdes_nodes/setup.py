from setuptools import find_packages, setup

package_name = 'vdes_nodes'

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
    maintainer='dev',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'py_target_generation = vdes_nodes.target_generation:main',
            'py_camera_simulation = vdes_nodes.camera_simulation:main',
            'py_gimbal_simulation = vdes_nodes.gimbal_simulation:main',
        ],
    },
)
