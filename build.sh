rm -rf dist/
# pyinstaller -F digitaltwin.py 
# cp -rf dist/digitaltwin digitaltwin_data/engines/bullet
rm -rf build/
python3 setup.py sdist
twine upload dist/practistyle-*
