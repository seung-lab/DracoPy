import setuptools
import os
import platform
import sys
import subprocess

import numpy as np

def read(fname):
  with open(os.path.join(os.path.dirname(__file__), fname), 'rt') as f:
    return f.read()

src_dir = './src'

setuptools.setup(
    name='DracoPy',
    version='1.4.1',
    description = 'Python wrapper for Google\'s Draco Mesh Compression Library',
    author = 'Manuel Castro, William Silversmith :: Contributors :: Fatih Erol, Faru Nuri Sonmez, Zeyu Zhao, Denis Riviere',
    author_email = 'macastro@princeton.edu, ws9@princeton.edu',
    url = 'https://github.com/seung-lab/DracoPy',
    long_description=read('README.md'),
    long_description_content_type="text/markdown",
    license = "License :: OSI Approved :: Apache Software License",
    setup_requires=['cython'],
    install_requires=['numpy'],
    ext_modules=[
        setuptools.Extension(
            'DracoPy',
            sources=[ os.path.join(src_dir, 'DracoPy.pyx') ],
            depends=[ os.path.join(src_dir, 'DracoPy.h') ],
            language='c++',
            include_dirs = [
                './draco/src/',
                np.get_include(),
            ],
            extra_compile_args=[
              '-std=c++11','-O3', '-stdlib=libc++'
            ],
            extra_link_args=[ "-ldraco" ],
        )
    ],
    classifiers=[
        "Intended Audience :: Developers",
        "Development Status :: 5 - Production/Stable",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering",
        "Operating System :: POSIX",
        "Operating System :: MacOS",
        "Operating System :: Microsoft :: Windows :: Windows 10",
        "Topic :: Utilities",
  ]
)
