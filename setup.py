#!/usr/bin/env python
# coding: utf-8
from setuptools import setup

setup(
    platforms=['win-amd64'],
    name='practools',
    version='0.0.1',
    author='sl.truman',
    author_email='sl.truman@live.com',
    url='',
    description='',
    packages=['practools','practools/end_effector'],
    install_requires=[
        'numpy',
        'scipy',
        'pybullet',
        'pygfx',
    ],
    include_package_data=True
)
