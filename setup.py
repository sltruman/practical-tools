#!/usr/bin/env python
# coding: utf-8

from setuptools import setup

setup(
    name='pysimflow',
    version='0.0.40',
    author='sl.truman',
    author_email='sl.truman@live.com',
    url='',
    description=u'',
    packages=['digitaltwin','digitaltwin_data'],
    install_requires=[
        'numpy',
        'pybullet',
        'scipy'
    ]
)
