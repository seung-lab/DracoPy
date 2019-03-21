import setuptools
import os
from skbuild import setup
from skbuild.constants import CMAKE_INSTALL_DIR
from packaging.version import LegacyVersion
from skbuild.exceptions import SKBuildError
from skbuild.cmaker import get_cmake_version

# Add CMake as a build requirement if cmake is not installed or is too low a version
setup_requires = []
try:
    if LegacyVersion(get_cmake_version()) < LegacyVersion("3.5"):
        setup_requires.append('cmake')
except SKBuildError:
    setup_requires.append('cmake')

# If you want to re-build the cython cpp file (DracoPy.cpp), run:
# cython --cplus -3 -I./_skbuild/linux-x86_64-3.6/cmake-install/include/draco/ ./src/DracoPy.pyx
# Draco must already be built/setup.py already be run before running the above command

src_dir = './src'
lib_dir = os.path.abspath(os.path.join(CMAKE_INSTALL_DIR(), 'lib/'))
library_link_args = ['-l:{0}'.format(lib) for lib in ('libdracoenc.a', 'libdraco.a', 'libdracodec.a')]
extra_link_args = ['-L{0}'.format(lib_dir)] + library_link_args

setup(
    name='DracoPy',
    version='0.0.2',
    description = 'Python wrapper for Google\'s Draco Mesh Compression Library',
    author = 'Manuel Castro',
    author_email = 'macastro@princeton.edu',
    url = 'https://github.com/seung-lab/DracoPy',
    cmake_source_dir='./draco',
    setup_requires=setup_requires,
    ext_modules=[
        setuptools.Extension(
            'DracoPy',
            sources=[ os.path.join(src_dir, 'DracoPy.cpp') ],
            depends=[ os.path.join(src_dir, 'DracoPy.h') ],
            language='c++',
            include_dirs = [ os.path.join(CMAKE_INSTALL_DIR(), 'include/')],
            extra_compile_args=[
              '-std=c++11','-O3'
            ],
            extra_link_args=extra_link_args
        )
    ]
)
