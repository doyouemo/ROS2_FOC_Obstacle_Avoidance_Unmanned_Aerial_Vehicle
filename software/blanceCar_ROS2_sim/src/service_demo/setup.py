from setuptools import find_packages, setup

package_name = 'service_demo'

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
    maintainer='doudou',
    maintainer_email='doudou@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'server_node = service_demo.server_node:main',
            # 'client_node = service_demo.client_node:main',
            'keyboard_control_node = service_demo.keyboard_control_node:main',
        ],
    },
)
