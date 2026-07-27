from setuptools import setup

package_name = 'openarmx_teach'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'rclpy', 'PyYAML'],
    zip_safe=True,
    maintainer='OpenArm User',
    maintainer_email='user@example.com',
    description='Utility scripts for recording and playing OpenArm joint states and trajectories.',
    license='CC-BY-NC-SA-4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_joint_states_stepbystep = openarmx_teach.record_joint_states_stepbystep:main',
            'record_joint_states_always = openarmx_teach.record_joint_states_always:main',
            'play_joint_trajectory = openarmx_teach.play_joint_trajectory:main',
        ],
    },
) 