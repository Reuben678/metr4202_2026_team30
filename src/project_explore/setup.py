import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'project_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name],),
        ('share/' + package_name, ['package.xml'],),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))],

    install_requires=[
        "setuptools",
        "networkx",
    ],
    zip_safe=True,
    maintainer='mjc',
    maintainer_email='mitchellcraw64@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'exploration_manager = project_explore.exploration_manager:main',
        ],
    },
)
