from setuptools import find_packages, setup

package_name = 'action_server_final'

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
    maintainer='alejo',
    maintainer_email='est.paula.calvo@unimilitar.edu.co',
    description='action server final',
    license='License Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'servidor = action_server_final.angle_action_server:main',
        	'cliente = action_server_final.angle_action_client:main',
        ],
    },
)
