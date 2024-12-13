from setuptools import find_packages, setup

package_name = 'rover_controller'

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
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "joystick_subscriber = rover_controller.joystick_subscriber:main",
            "rover_joy_translator = rover_controller.rover_joy_translator:main"
        ],
    },
)
