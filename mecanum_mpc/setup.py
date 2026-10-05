from glob import glob

from setuptools import find_packages, setup

package_name = 'mecanum_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varad',
    description='LTV-MPC path tracking for the own mecanum robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = mecanum_mpc.mpc_node:main',
            'offline_sim = mecanum_mpc.offline_sim:main',
            'evaluate = mecanum_mpc.evaluate:main',
        ],
    },
)
