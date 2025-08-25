from setuptools import find_packages, setup

package_name = 'my_tf'

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
            'my_tf_1 = my_tf.my_tf_1:main',
            'my_tf_2 = my_tf.my_tf_2:main',
            'child_frame = my_tf.child_frame:main',
            'combined_frame = my_tf.combined_frame:main',
            'distance_world_child_publisher = my_tf.distance_world_child_publisher:main',
        ],
    },
)
