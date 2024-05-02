from setuptools import find_packages, setup

package_name = 'turtlebot3_stop_sign'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Correct path for the XML file
    ('share/' + package_name, ['turtlebot3_stop_sign/stop_sign_classifier_2.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sj',
    maintainer_email='sj@todo.todo',
    description='A ROS2 package to make TurtleBot3 stop at stop signs using OpenCV and Haar Cascades',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_sign_detector = turtlebot3_stop_sign.stop_sign_detector:main'
        ],
    },
)
