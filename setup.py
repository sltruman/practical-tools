#!/usr/bin/env python
# coding: utf-8
from setuptools import setup


def get_version():
    import re
    result = re.search(r'(\d+)\.(\d+)\.(\d+)', open('version.in').read())
    major = result.group(1)
    minor = result.group(2)
    patch = int(result.group(3)) + 1
    version = f'{major}.{minor}.{patch}'
    open('version.in','w').write(version)
    return version

setup(
    platforms=['win-amd64'],
    name='practistyle',
    version=get_version(),
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
