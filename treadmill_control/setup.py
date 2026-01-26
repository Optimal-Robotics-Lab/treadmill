from setuptools import find_packages, setup

package_name = 'treadmill_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pymodbus',],
    zip_safe=True,
    maintainer='ciscor',
    maintainer_email='ciscor@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'treadmill_node = treadmill_control.treadmill_node:main'
        ],
    },
)
