from setuptools import setup

package_name = 'tcp_test'

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
    maintainer='infrastructure-3',
    maintainer_email='luke.strohbehn@gmail.com',
    description='Double publisher node to communicate with Arduino via TCP/IP socket.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linear_slider_node = tcp_test.linear_slider_node:main',
            'tcp_pos_publisher = tcp_test.tcp_pos_publisher:main',
            'tcp_target_publisher = tcp_test.tcp_target_publisher:main'
        ],
    },
)
