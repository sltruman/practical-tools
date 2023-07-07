rm -rf dist/
pyinstaller -F backend.py 
cp -rf dist/backend practistyle/data/engines/backend
rm -rf build/
python3 setup.py sdist
twine upload dist/practistyle-*
