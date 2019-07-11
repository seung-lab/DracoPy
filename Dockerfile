FROM quay.io/pypa/manylinux2010_x86_64
MAINTAINER Manuel Castro

COPY . /DracoPy

WORKDIR "/DracoPy"

ENV GCC "g++"

RUN rm -rf *.so _skbuild __pycache__ dist DracoPy.egg-info

RUN /opt/python/cp27-cp27m/bin/pip2.7 install pip --upgrade

RUN /opt/python/cp27-cp27m/bin/pip2.7 install scikit-build twine

RUN /opt/python/cp27-cp27m/bin/python2.7 setup.py develop

RUN /opt/python/cp27-cp27m/bin/python2.7 setup.py sdist bdist_wheel

RUN for whl in `ls dist/*.whl`; do auditwheel repair --plat manylinux2010_x86_64 $whl; done
