import setuptools
import os
import platform
import sys

from skbuild import setup
from skbuild.constants import CMAKE_INSTALL_DIR, skbuild_plat_name
from packaging.version import LegacyVersion
from skbuild.exceptions import SKBuildError
from skbuild.cmaker import get_cmake_version

import numpy as np
import multiprocessing as mp

if not "CMAKE_BUILD_PARALLEL_LEVEL" in os.environ:
    os.environ["CMAKE_BUILD_PARALLEL_LEVEL"] = str(mp.cpu_count())

def read(fname):
  with open(os.path.join(os.path.dirname(__file__), fname), 'rt') as f:
    return f.read()

# Add CMake as a build requirement if cmake is not installed or is too low a version
setup_requires = []
try:
    cmake_version = LegacyVersion(get_cmake_version())
    if cmake_version < LegacyVersion("3.5") or cmake_version >= LegacyVersion("3.15"):
        setup_requires.append('cmake<3.15')
except SKBuildError:
    setup_requires.append('cmake<3.15')

# If you want to re-build the cython cpp file (DracoPy.cpp), run:
# cython --cplus -3 -I./_skbuild/linux-x86_64-3.6/cmake-install/include/draco/ ./src/DracoPy.pyx
# Replace "linux-x86_64-3.6" with the directory under _skbuild in your system
# Draco must already be built/setup.py already be run before running the above command

src_dir = './src'
lib_dir = os.path.abspath(os.path.join(CMAKE_INSTALL_DIR(), 'lib/'))
cmake_args = []

operating_system = platform.system().lower()

is_macos = sys.platform == 'darwin' or operating_system == "darwin"
is_windows = sys.platform == 'win32' or operating_system == "windows"

if is_macos:
    plat_name = skbuild_plat_name()
    sep = [pos for pos, char in enumerate(plat_name) if char == '-']
    assert len(sep) == 2
    cmake_args = [
        '-DCMAKE_OSX_DEPLOYMENT_TARGET:STRING='+plat_name[sep[0]+1:sep[1]],
        '-DCMAKE_OSX_ARCHITECTURES:STRING='+plat_name[sep[1]+1:]
    ]
    library_link_args = [
        f'-l{lib}' for lib in ('dracoenc', 'draco', 'dracodec')
    ]
elif is_windows:
    library_link_args = [ 
        lib for lib in ('dracoenc.lib', 'draco.lib', 'dracodec.lib')
    ]
else: # linux
    library_link_args = [
        f'-l:{lib}' for lib in ('libdracoenc.a', 'libdraco.a', 'libdracodec.a')
    ]

if is_windows:
    extra_link_args = ['/LIBPATH:{0}'.format(lib_dir)] + library_link_args
    extra_compile_args = [
      '/std:c++17', '/O2',
    ]
else:
    extra_link_args = ['-L{0}'.format(lib_dir)] + library_link_args
    extra_compile_args = [
      '-std=c++11','-O3'
    ]

setup(
    name='DracoPy',
    version='1.0.1',
    description = 'Python wrapper for Google\'s Draco Mesh Compression Library',
    author = 'Manuel Castro, William Silversmith :: Contributors :: Fatih Erol, Faru Nuri Sonmez',
    author_email = 'macastro@princeton.edu, ws9@princeton.edu',
    url = 'https://github.com/seung-lab/DracoPy',
    long_description=read('README.md'),
    long_description_content_type="text/markdown",
    license = "License :: OSI Approved :: Apache Software License",
    cmake_source_dir='./draco',
    cmake_args=cmake_args,
    setup_requires=setup_requires,
    install_requires=['pytest'],
    ext_modules=[
        setuptools.Extension(
            'DracoPy',
            sources=[ os.path.join(src_dir, 'DracoPy.cpp') ],
            depends=[ os.path.join(src_dir, 'DracoPy.h') ],
            language='c++',
            include_dirs = [ 
                np.get_include(),
                os.path.join(CMAKE_INSTALL_DIR(), 'include/'),
            ],
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args
        )
    ],
    classifiers=[
        "Intended Audience :: Developers",
        "Development Status :: 5 - Production/Stable",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Topic :: Scientific/Engineering",
        "Operating System :: POSIX",
        "Operating System :: MacOS",
        "Operating System :: Microsoft :: Windows :: Windows 10",
        "Topic :: Utilities",
  ]
)
