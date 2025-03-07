from setuptools import setup

package_name = 'f1tenth_espol'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'cruise = f1tenth_espol.cruise:main',
        'keyboard = f1tenth_espol.keyboard:main',
        'keyboard_teleop = f1tenth_espol.keyboard_teleop:main',
        'gap_follower = f1tenth_espol.gap_follower:main'
        ],
    },
)
