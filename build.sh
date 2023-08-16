rm -rf dist/
pyinstaller -F backend.py 
cp -rf dist/backend practical-room/data/engines/backend
rm -rf build/
python3 setup.py sdist
twine upload dist/practical-room*
