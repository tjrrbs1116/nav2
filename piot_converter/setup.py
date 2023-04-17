import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'piot_converter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='piot',
    author_email='rkd1037@piot.co.kr',
    maintainer='piot',
    maintainer_email='rkd103@piot.co.kr',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Converter node for PIOT robot.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piot_converter = piot_converter.script.piot_converter:main'
        ],
    },
)
