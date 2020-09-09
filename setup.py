import setuptools
import os
import sys
from skbuild import setup
from skbuild.constants import CMAKE_INSTALL_DIR, skbuild_plat_name
from packaging.version import LegacyVersion
from skbuild.exceptions import SKBuildError
from skbuild.cmaker import get_cmake_version

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
if sys.platform == 'darwin':
    plat_name = skbuild_plat_name()
    sep = [pos for pos, char in enumerate(plat_name) if char == '-']
    assert len(sep) == 2
    cmake_args = ['-DCMAKE_OSX_DEPLOYMENT_TARGET:STRING='+plat_name[sep[0]+1:sep[1]],'-DCMAKE_OSX_ARCHITECTURES:STRING='+plat_name[sep[1]+1:]]
    library_link_args = ['-l{0}'.format(lib) for lib in ('dracoenc', 'draco', 'dracodec')]
elif sys.platform == 'win32':
    library_link_args = ['{0}'.format(lib) for lib in ('dracoenc.lib', 'draco.lib', 'dracodec.lib')]
else:
    library_link_args = ['-l:{0}'.format(lib) for lib in ('libdracoenc.a', 'libdraco.a', 'libdracodec.a')]

if sys.platform == 'win32':
    extra_link_args = ['/LIBPATH:{0}'.format(lib_dir)] + library_link_args
    extra_compile_args = [
              '/std:c++17','-O3'
            ]
else:
    extra_link_args = ['-L{0}'.format(lib_dir)] + library_link_args
    extra_compile_args = [
              '-std=c++11','-O3'
            ]


setup(
    name='DracoPy',
    version='0.0.15',
    description = 'Python wrapper for Google\'s Draco Mesh Compression Library',
    author = 'Manuel Castro',
    author_email = 'macastro@princeton.edu',
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
            include_dirs = [ os.path.join(CMAKE_INSTALL_DIR(), 'include/')],
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
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Topic :: Scientific/Engineering",
        "Operating System :: POSIX",
        "Operating System :: MacOS",
        "Operating System :: Microsoft :: Windows :: Windows 10",
        "Topic :: Utilities",
  ]
)
