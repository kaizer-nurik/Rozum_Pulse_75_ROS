from setuptools import find_packages, setup

package_name = 'simple_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +"config", ['config/conf.yaml']),
        ('lib/' + package_name , ['simple_demo/demo.py']),
        ('share/' + package_name , ['launch/demo.launch.py']),
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
            'demo = simple_demo.demo:main',
        ],
    },
)
