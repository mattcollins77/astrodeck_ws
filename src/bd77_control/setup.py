from setuptools import find_packages, setup

package_name = 'bd77_control'

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
    maintainer='matt',
    maintainer_email='matthew_collins@icloud.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_input = bd77_control.publisher_keyboard_input:main',
            'sound_control = bd77_control.sound_control:main',
            'eye_control = bd77_control.eye_control:main',
            'ear_control = bd77_control.ear_control:main',
            'steamdeck_input = bd77_control.steamdeck_input_subscriber:main',
            'head_movement_control = bd77_control.head_movement_control:main'
        ],
    },
)
