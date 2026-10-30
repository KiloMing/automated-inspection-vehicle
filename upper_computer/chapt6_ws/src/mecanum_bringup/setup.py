from glob import glob
from setuptools import find_packages, setup

package_name = 'mecanum_bringup'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + [('share/' + package_name + '/' + folder, glob(folder + '/*'))
         for folder in ('launch', 'config', 'urdf', 'worlds', 'scripts')],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='KiloMing', maintainer_email='3402684841@qq.com',
    description='Independent Jazzy mecanum simulation and integration bringup',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'motion_smoke = mecanum_bringup.motion_smoke:main',
    ]},
)
