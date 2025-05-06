from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='ldl',
    maintainer_email='lidelong@agibot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.joint_state_listener_node:main',
                'mujoco_ctrl = py_pubsub.mujoco_ctrl:main',
                'mujoco_ctrl_by_joint.py = py_pubsub.mujoco_ctrl_by_joint:main',

        ],
    },
)
