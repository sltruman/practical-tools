#!/usr/bin/env python
# coding: utf-8
from setuptools import setup
import re
result = re.search(r'(\d+)\.(\d+)\.(\d+)', open('__init__.py').read())
major = result.group(1)
minor = result.group(2)
patch = int(result.group(3)) + 1
version = f'{major}.{minor}.{patch}'
open('__init__.py','w').write(f"__version__='{version}'")

setup(
    name='pysimflow',
    version=version,
    author='sl.truman',
    author_email='sl.truman@live.com',
    url='',
    description=u'',
    packages=['digitaltwin','digitaltwin/end_effector','py3dbp','digitaltwin_data'],
    install_requires=[
        'numpy',
        'pybullet',
        'scipy'
    ],
    include_package_data=True
)
