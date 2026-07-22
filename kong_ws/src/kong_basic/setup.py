from setuptools import find_packages, setup
from glob import glob
import os
from setuptools import find_packages

package_name = 'kong_basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob(os.path.join("launch",
                                                                 "*.launch.py")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ju',
    maintainer_email='jyber9616@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "simple_pub = kong_basic.simple_pub:main",
            "class_pub = kong_basic.class_pub:main",
            "class_sub = kong_basic.class_sub:main",
            "header_pub = kong_basic.header_pub:main",
            "time_sub = kong_basic.time_sub:main",
            "mv_turtle = kong_basic.mv_turtle:main",
            "drift_turtle = kong_basic.drift_turtle:main",
            "qos_test_pub = kong_basic.qos_test_pub:main",
            "qos_test_sub = kong_basic.qos_test_sub:main",
        ],
    },
)
