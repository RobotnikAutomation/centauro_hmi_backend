from setuptools import setup


package_name = 'centauro_hmi_ws_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.common'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ws_send_command_demo = centauro_hmi_ws_examples.send_command_demo:main',
            'ws_teleoperation_demo = centauro_hmi_ws_examples.teleoperation_demo:main',
            'ws_jog_for_seconds_demo = centauro_hmi_ws_examples.jog_for_seconds_demo:main',
            'ws_execute_pending_trajectory_demo = centauro_hmi_ws_examples.execute_pending_trajectory_demo:main',
            'ws_robot_model_demo = centauro_hmi_ws_examples.robot_model_demo:main',
            'ws_plan_joint_trajectory_demo = centauro_hmi_ws_examples.plan_joint_trajectory_demo:main',
            'ws_cancel_operation_demo = centauro_hmi_ws_examples.cancel_operation_demo:main',
            'ws_protocol_smoke_test = centauro_hmi_ws_examples.protocol_smoke_test:main',
        ],
    },
)
