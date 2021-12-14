[![PyPI version](https://badge.fury.io/py/DracoPy.svg)](https://badge.fury.io/py/DracoPy)
[![Build Status](https://travis-ci.org/seung-lab/DracoPy.svg?branch=master)](https://travis-ci.org/seung-lab/DracoPy)

# DracoPy

```python
import os
import DracoPy

with open('bunny.drc', 'rb') as draco_file:
  mesh = DracoPy.decode(draco_file.read())

print(f"number of points: {len(mesh.points)}")
print(f"number of faces: {len(mesh.faces)}")
print(f"number of normals: {len(mesh.normals)}")

# Note: If mesh.points is an integer numpy array,
# it will be encoded as an integer attribute. Otherwise,
# it will be encoded as floating point.
binary = DracoPy.encode(mesh.points, mesh.faces)
with open('bunny_test.drc', 'wb') as test_file:
  test_file.write(encoding_test)

```

DracoPy is a Python wrapper for Google's Draco mesh compression library.

## Installation

Binary wheels are available for users with Python >= 3.6 and pip >= 20.

Installation from source requires Python >= 3.6, pip >= 10, and a C++ compiler that is fully compatible with C++11.

It supports Linux, OS X, and Windows. Numpy is required.

```bash
pip install DracoPy 
```

## Acknowledgements 

We graciously thank The Stanford 3D Scanning Repository for providing the Stanford Bunny test model. 

https://graphics.stanford.edu/data/3Dscanrep/


