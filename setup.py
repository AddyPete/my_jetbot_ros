from setuptools import setup

package_name = 'my_jetbot_ros'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/jetbot_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_jetbot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csu-summo-minegears',
    maintainer_email='csu-summo-minegears@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetbot_driver = my_jetbot_ros.jetbot_driver:main',
            'odom_estimator = my_jetbot_ros.odom_estimator:main'
        ],
    },
)
