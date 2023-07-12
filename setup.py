#!/usr/bin/env python
# coding: utf-8
from setuptools import setup

setup(
    platforms=['win-amd64'],
    name='practistyle',
    version='0.0.78',
    author='sl.truman',
    author_email='sl.truman@live.com',
    url='',
    description=u'',
    packages=['practistyle','practistyle/end_effector','py3dbp','practistyle/data'],
    install_requires=[
        'numpy',
        'scipy',
        'pybullet',
    ],
    include_package_data=True
)
