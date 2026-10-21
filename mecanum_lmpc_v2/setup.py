from glob import glob

from setuptools import find_packages, setup

package_name = 'mecanum_lmpc_v2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varad',
    description='Linear MPC v2 for the own mecanum robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lmpc_v2_node = mecanum_lmpc_v2.lmpc_v2_node:main',
            'offline_sim = mecanum_lmpc_v2.offline_sim:main',
            'sign_test = mecanum_lmpc_v2.sign_test:main',
        ],
    },
)
