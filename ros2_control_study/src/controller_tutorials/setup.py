from setuptools import find_packages, setup

package_name = 'controller_tutorials'

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
    maintainer='jinhyuk2me',
    maintainer_email='164614732+jinhyuk2me@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_rotate = controller_tutorials.simple_rotate:main',
            'control_rotate = controller_tutorials.control_rotate:main',
            'control_apps = controller_tutorials.control_apps:main',
            'pose_dual_controller = controller_tutorials.pose_dual_controller:main',
            'qmonitor_for_pose_dual_controller = controller_tutorials.qmonitor_for_pose_dual_controller:main',
            'move_turtle = controller_tutorials.move_turtle:main',
            'monitor_for_move_turtle = controller_tutorials.monitor_for_move_turtle:main',
        ],
    },
)
