from setuptools import find_packages, setup

package_name = 'realsense_to_scene_transform'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/static_tf.yaml']),
        ('share/' + package_name + '/config', ['config/d435_d455_aruco_params.yaml']),
        ('lib/' + package_name , ['realsense_to_scene_transform/static_tf_node.py']),
        ('share/' + package_name + '/rviz', ['rviz/cam_detect.rviz']),
        ('share/' + package_name + '/launch', ['launch/dual_aruco_with_tf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panzer',
    maintainer_email='kerimovnm@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_yaml = realsense_to_scene_transform.static_tf_node:main',
            'static_tf_node = realsense_to_scene_transform.static_tf_node:main',
            'static_tf_node.py = realsense_to_scene_transform.static_tf_node:main',
        ],
    },
)
