from setuptools import find_packages
from setuptools import setup

setup(
    name='abu_core',
    version='2.1.5',
    packages=find_packages(
        include=('abu_core', 'abu_core.*')),
)
