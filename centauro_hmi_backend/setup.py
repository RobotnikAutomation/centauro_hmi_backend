from setuptools import setup

package_name = 'centauro_hmi_backend'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.robots'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/backend.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/backend.launch.py',
            'launch/mock_visualization.launch.py',
            'launch/complete.launch.py',
        ]),
        ('share/' + package_name + '/urdf', ['urdf/arm.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/arm.rviz']),
        ('share/' + package_name + '/meshes', [
            'meshes/base_link.stl',
            'meshes/shoulder_link.stl',
            'meshes/upper_arm_link.stl',
            'meshes/forearm_link.stl',
            'meshes/wrist_1_link.stl',
            'meshes/wrist_2_link.stl',
            'meshes/wrist_3_link.stl',
            'meshes/tool0.stl',
        ]),
    ],
    install_requires=['setuptools', 'websockets>=9.1,<11'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi_backend = centauro_hmi_backend.node:main',
        ],
    },
)
