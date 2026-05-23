from setuptools import find_packages, setup

package_name = 'moveit_control'

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
    maintainer='gugugaga',
    maintainer_email='gugugaga@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_pose = moveit_control.move_to_pose:main',
            'check_collision = moveit_control.check_collision:main',
            'move_in_safety = moveit_control.move_in_safety:main',
            'moveit_pose_server = moveit_control.moveit_pose_server:main',
            'target_pose_gui = moveit_control.target_pose_gui:main',
    ],
},
)
