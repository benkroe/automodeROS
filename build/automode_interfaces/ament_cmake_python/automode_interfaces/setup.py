from setuptools import find_packages
from setuptools import setup

setup(
    name='automode_interfaces',
    version='1.0.0',
    packages=find_packages(
        include=('automode_interfaces', 'automode_interfaces.*')),
)
