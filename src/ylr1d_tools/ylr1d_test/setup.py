import os

from setuptools import find_packages, setup

package_name = 'ylr1d_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            [os.path.join('launch', f) for f in os.listdir('launch')
             if f.endswith('.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zsj',
    maintainer_email='2607430020@qq.com',
    description='YLR1D functional test package (unified smoke tests)',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ylr1d-test = ylr1d_test.run_all:main',
        ],
    },
)
