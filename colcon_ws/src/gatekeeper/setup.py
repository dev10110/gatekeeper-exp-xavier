from setuptools import setup

package_name = 'gatekeeper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'decomp_ros_msgs'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_gatekeeper = gatekeeper.simple_gatekeeper:main',
            'gatekeeper = gatekeeper.gatekeeper:main',
            'mpc = gatekeeper.mpc:main',
            'blind_follow = gatekeeper.blind_follow:main',
        ],
    },
)
