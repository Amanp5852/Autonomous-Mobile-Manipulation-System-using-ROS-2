from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='aman',
    maintainer_email='aman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "navigation_node = my_robot_controller.navigation_node:main",
            "manipulation_node = my_robot_controller.manipulation_node:main",
            "camera_node = my_robot_controller.camera_node:main",
            "task_manager_node = my_robot_controller.task_manager_node:main",
            "navigation_node_v2 = my_robot_controller.navigation_node_v2:main"
        ],
    },
)
