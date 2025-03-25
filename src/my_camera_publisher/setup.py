from setuptools import setup

package_name = 'my_camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aav',
    maintainer_email='your-email@example.com',
    description='Camera Publisher for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = my_camera_publisher.image_publisher:main'
        ],
    },
)

