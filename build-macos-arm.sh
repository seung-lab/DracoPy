#!/bin/sh

function build {
	python setup-macos-arm.py develop
	python -m pytest -v -x tests.py
	python setup-macos-arm.py bdist_wheel	
}

rm -r .eggs build *.so
make 

for venv in dracopy38 dracopy39 dracopy310 dracopy311
do
	echo $venv
	workon $venv
	build
done

workon dracopy