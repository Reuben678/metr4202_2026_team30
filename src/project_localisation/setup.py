from importlib.metadata import entry_points

from setuptools import find_packages, setup

package_name = 'project_localisation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'LICENCE',
    ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mjc',
    maintainer_email='mitchellcraw64@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest','flake8','pydocstyle'
        ],
    },
    entry_points={
        'console_scripts':[
            'localizer = project_localisation.target_localisation:localizer_main',
            'evaluate = project_localisation.evaluate:evaluate_main'
        ]
    }

)



