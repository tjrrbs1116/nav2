import os
from glob import glob
from setuptools import setup

package_name = 'piot_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('lib', package_name), glob(package_name + '/rtmgr.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PIOT',
    maintainer_email='rkd1037@piot.co.kr',
    description='PIOT UI node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piot_ui_node = piot_ui.piot_ui_dialog:main',
        ],
    },
)
