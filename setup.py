from setuptools import find_packages, setup

package_name = 'webots_ros2_limo'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/protos', ['protos/LimoFourDiff.proto']))
data_files.append(('share/' + package_name + '/protos/meshes', [
    'protos/meshes/limo_base.dae',
    'protos/meshes/limo_wheel.dae',
]))
data_files.append(('share/' + package_name + '/worlds', ['worlds/limo_world.wbt']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/ros2control.yaml',
    'resource/limo.urdf',
    'resource/nav2_params_limo.yaml',
    'resource/limo_lds_2d.lua',
    'resource/limo_example_map.yaml',
    'resource/limo_example_map.pgm',
]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
    'launch/cartographer_launch.py',
    'launch/nav2_launch.py',
]))
data_files.append(('share/' + package_name + '/rviz', ['rviz/limo_cartographer.rviz']))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inmojang',
    maintainer_email='inmo.jang@kau.ac.kr',
    description='AgileX LIMO (four-wheel diff) Webots simulation with webots_ros2_driver + ros2_control + Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
