from setuptools import find_packages, setup

package_name = 'mecanum_mpc_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varad',
    description='Shared base of the own mecanum robot controllers',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evaluate = mecanum_mpc_common.evaluate:main',
            'plot_runs = mecanum_mpc_common.plot_runs:main',
            'record_video = mecanum_mpc_common.record_video:main',
        ],
    },
)
