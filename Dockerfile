FROM quay.io/pypa/manylinux2014_aarch64
MAINTAINER William Silversmith

COPY . /DracoPy

WORKDIR "/DracoPy"

ENV GCC "gcc"

RUN rm -rf *.so _skbuild __pycache__ dist DracoPy.egg-info

RUN yum update && yum install -y openssl-devel cmake

RUN /opt/python/cp38-cp38/bin/pip3.8 install pip --upgrade
RUN /opt/python/cp39-cp39/bin/pip3.9 install pip --upgrade
RUN /opt/python/cp310-cp310/bin/pip3.10 install pip --upgrade
RUN /opt/python/cp311-cp311/bin/pip3.11 install pip --upgrade
RUN /opt/python/cp312-cp312/bin/pip3.12 install pip --upgrade

RUN /opt/python/cp38-cp38/bin/pip3.8 install scikit-build twine numpy cython pytest -U
RUN /opt/python/cp39-cp39/bin/pip3.9 install scikit-build twine numpy cython pytest -U 
RUN /opt/python/cp310-cp310/bin/pip3.10 install scikit-build twine numpy cython pytest -U
RUN /opt/python/cp311-cp311/bin/pip3.11 install scikit-build twine numpy cython pytest -U
RUN /opt/python/cp312-cp312/bin/pip3.12 install scikit-build twine numpy cython pytest -U

RUN touch src/DracoPy.pyx && /opt/python/cp38-cp38/bin/python3.8 setup.py develop
RUN touch src/DracoPy.pyx && /opt/python/cp39-cp39/bin/python3.9 setup.py develop
RUN touch src/DracoPy.pyx && /opt/python/cp310-cp310/bin/python3.10 setup.py develop
RUN touch src/DracoPy.pyx && /opt/python/cp311-cp311/bin/python3.11 setup.py develop
RUN touch src/DracoPy.pyx && /opt/python/cp312-cp312/bin/python3.12 setup.py develop

RUN /opt/python/cp38-cp38/bin/python3.8 -m pytest -v -x tests.py
RUN /opt/python/cp39-cp39/bin/python3.9 -m pytest -v -x tests.py
RUN /opt/python/cp310-cp310/bin/python3.10 -m pytest -v -x tests.py
RUN /opt/python/cp311-cp311/bin/python3.11 -m pytest -v -x tests.py
RUN /opt/python/cp312-cp312/bin/python3.12 -m pytest -v -x tests.py

RUN touch src/DracoPy.pyx && /opt/python/cp38-cp38/bin/python3.8 setup.py bdist_wheel
RUN touch src/DracoPy.pyx && /opt/python/cp39-cp39/bin/python3.9 setup.py bdist_wheel
RUN touch src/DracoPy.pyx && /opt/python/cp310-cp310/bin/python3.10 setup.py bdist_wheel
RUN touch src/DracoPy.pyx && /opt/python/cp311-cp311/bin/python3.11 setup.py bdist_wheel
RUN touch src/DracoPy.pyx && /opt/python/cp312-cp312/bin/python3.12 setup.py bdist_wheel

RUN for whl in `ls dist/*.whl`; do auditwheel repair --plat manylinux2014_aarch64 $whl; done
