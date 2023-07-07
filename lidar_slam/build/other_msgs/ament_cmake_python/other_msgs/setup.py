from setuptools import find_packages
from setuptools import setup

setup(
    name='other_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('other_msgs', 'other_msgs.*')),
)
