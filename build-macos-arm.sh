#!/bin/zsh

source /usr/local/bin/virtualenvwrapper.sh

export ARCHFLAGS="-arch arm64"

function build {
	pip install numpy cython -U
	rm src/DracoPy.cpp
	cython -3 src/DracoPy.pyx
	python setup-macos-arm.py develop
	python -m pytest -v -x tests.py
	python setup-macos-arm.py bdist_wheel	
}

rm -r .eggs build *.so
make -j 7

for venv in dracopy38 dracopy39 dracopy310 dracopy311 dracopy312;
do
	echo $venv
	workon $venv
	build
done

workon dracopy