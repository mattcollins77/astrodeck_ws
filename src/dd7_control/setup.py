from setuptools import find_packages, setup

package_name = 'dd7_control'

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
            'my_node = dd7_control.my_node:main',
            'talker = dd7_control.publisher_member_function:main',
            'listener = dd7_control.subscriber_member_function:main',
        ],
    },
)