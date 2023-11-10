#!/bin/bash
rm -rf dist/
rm -rf build/
python3 setup.py sdist
twine upload dist/practools*