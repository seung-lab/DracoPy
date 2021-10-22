[![PyPI version](https://badge.fury.io/py/DracoPy.svg)](https://badge.fury.io/py/DracoPy)
[![Build Status](https://travis-ci.org/seung-lab/DracoPy.svg?branch=master)](https://travis-ci.org/seung-lab/DracoPy)

# DracoPy

```python
import os
import DracoPy

with open('bunny.drc', 'rb') as draco_file:
  file_content = draco_file.read()
  mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
  print('number of points in original file: {0}'.format(len(mesh_object.points)))
  print('number of faces in original file: {0}'.format(len(mesh_object.faces)))
  encoding_test = DracoPy.encode_mesh_to_buffer(mesh_object.points, mesh_object.faces)
  with open('bunny_test.drc', 'wb') as test_file:
    test_file.write(encoding_test)

with open('bunny_test.drc', 'rb') as test_file:
  file_content = test_file.read()
  mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
  print('number of points in test file: {0}'.format(len(mesh_object.points)))
  print('number of faces in test file: {0}'.format(len(mesh_object.faces)))
```

DracoPy is a Python wrapper for Google's Draco mesh compression library.

## Installation

Binary wheels are available for users with Python >= 3.6 and pip >= 20.

Installation from source requires Python >= 3.6, pip >= 10, and a C++ compiler that is fully compatible with C++11.

It supports Linux, OS X, and Windows. 

```bash
pip install DracoPy 
```



