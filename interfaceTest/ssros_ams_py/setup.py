from setuptools import setup

package_name = 'ssros_ams_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Python ROS2 examples for messages, services, and actions.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_publisher = ssros_ams_py.gesture_publisher:main',
            'gesture_subscriber = ssros_ams_py.gesture_subscriber:main',
            'set_fan_pwm_server = ssros_ams_py.set_fan_pwm_server:main',
            'set_fan_pwm_client = ssros_ams_py.set_fan_pwm_client:main',
            'spin_action_server = ssros_ams_py.spin_action_server:main',
            'spin_action_client = ssros_ams_py.spin_action_client:main',
        ],
    },
)
