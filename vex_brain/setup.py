from setuptools import setup

package_name = 'vex_brain'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = vex_brain.talker:main',
        	'test_balance = vex_brain.test_balance:main',
        	'motor = vex_brain.vex_motor:main',
        	'rotation_sensor = vex_brain.vex_rotation_sensor:main',
        	'screen = vex_brain.vex_screen:main',
        	'inertial = vex_brain.vex_inertial:main',
        	'threewire = vex_brain.vex_threewire:main'
        ],
    },
)
