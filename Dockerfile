FROM quay.io/pypa/manylinux2010_x86_64
MAINTAINER Manuel Castro

COPY . /DracoPy

WORKDIR "/DracoPy"

ENV GCC "g++"

RUN rm -rf *.so _skbuild __pycache__ dist DracoPy.egg-info

RUN /opt/python/cp36-cp36m/bin/pip3.6 install pip --upgrade
RUN /opt/python/cp37-cp37m/bin/pip3.7 install pip --upgrade
RUN /opt/python/cp38-cp38/bin/pip3.8 install pip --upgrade
RUN /opt/python/cp39-cp39/bin/pip3.9 install pip --upgrade

RUN /opt/python/cp36-cp36m/bin/pip3.6 install scikit-build twine
RUN /opt/python/cp37-cp37m/bin/pip3.7 install scikit-build twine
RUN /opt/python/cp38-cp38/bin/pip3.8 install scikit-build twine
RUN /opt/python/cp39-cp39/bin/pip3.9 install scikit-build twine

RUN /opt/python/cp36-cp36m/bin/python3.6 setup.py develop
RUN /opt/python/cp37-cp37m/bin/python3.7 setup.py develop
RUN /opt/python/cp38-cp38/bin/python3.8 setup.py develop
RUN /opt/python/cp39-cp39/bin/python3.9 setup.py develop

RUN /opt/python/cp36-cp36m/bin/python3.6 -m pytest -v -x tests.py
RUN /opt/python/cp37-cp37m/bin/python3.7 -m pytest -v -x tests.py
RUN /opt/python/cp38-cp38/bin/python3.8 -m pytest -v -x tests.py
RUN /opt/python/cp39-cp39/bin/python3.9 -m pytest -v -x tests.py

RUN /opt/python/cp36-cp36m/bin/python3.6 setup.py sdist bdist_wheel
RUN /opt/python/cp37-cp37m/bin/python3.7 setup.py sdist bdist_wheel
RUN /opt/python/cp38-cp38/bin/python3.8 setup.py sdist bdist_wheel
RUN /opt/python/cp39-cp39/bin/python3.9 setup.py sdist bdist_wheel

RUN for whl in `ls dist/*.whl`; do auditwheel repair --plat manylinux2010_x86_64 $whl; done
