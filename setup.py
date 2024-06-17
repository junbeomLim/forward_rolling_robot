from setuptools import find_packages, setup

package_name = 'forward_rolling_robot'

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
    maintainer='limjunbeom',
    maintainer_email='ljbliblib@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = forward_rolling_robot.publisher_member_function:main',
            'listener = forward_rolling_robot.subscriber_member_function:main',
            'get_keyboard = forward_rolling_robot.get_keyboard:main',
            'run_robot = forward_rolling_robot.run_robot:main',
        ],
    },
)
