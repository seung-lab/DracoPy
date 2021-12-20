# distutils: language = c++
from typing import Union, cast

from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport DracoPy
import struct
from math import floor
from libc.string cimport memcmp
from libc.stdint cimport (
  int8_t, int16_t, int32_t, int64_t,
  uint8_t, uint16_t, uint32_t, uint64_t,
)

cimport numpy as np
import numpy as np

class DracoPointCloud:
    def __init__(self, data_struct):
        self.data_struct = data_struct
        if data_struct['encoding_options_set']:
            self.encoding_options = EncodingOptions(data_struct['quantization_bits'],
                data_struct['quantization_range'], data_struct['quantization_origin'])
        else:
            self.encoding_options = None
    
    def get_encoded_coordinate(self, value, axis):
        if self.encoding_options is not None:
            return self.encoding_options.get_encoded_coordinate(value, axis)

    def get_encoded_point(self, point):
        if self.encoding_options is not None:
            return self.encoding_options.get_encoded_point(point)

    @property
    def num_axes(self):
        return 3

    @property
    def points(self):
        points = self.data_struct['points']
        N = len(points) // 3
        return np.array(points).reshape((N, 3))

class DracoMesh(DracoPointCloud):
    @property
    def faces(self):
        faces_ = self.data_struct['faces']
        N = len(faces_) // 3
        return np.array(faces_).reshape((N, 3))

    @property
    def normals(self):
        normals_ = self.data_struct['normals']
        N = len(normals_) // 3
        return np.array(normals_).reshape((N,3))

class EncodingOptions(object):
    def __init__(self, quantization_bits, quantization_range, quantization_origin):
        self.quantization_bits = quantization_bits
        self.quantization_range = quantization_range
        self.quantization_origin = quantization_origin
        self.inverse_alpha = quantization_range / ((2 ** quantization_bits) - 1)
    
    def get_encoded_coordinate(self, value, axis):
        if value < self.quantization_origin[axis] or value > (self.quantization_origin[axis] + self.quantization_range):
            raise ValueError('Specified value out of encoded range')
        difference = value - self.quantization_origin[axis]
        quantized_index = floor((difference / self.inverse_alpha) + 0.5)
        return self.quantization_origin[axis] + (quantized_index * self.inverse_alpha)

    def get_encoded_point(self, point):
        encoded_point = []
        for axis in range(self.num_axes):
            encoded_point.append(self.get_encoded_coordinate(point[axis], axis))
        return encoded_point
    
    @property
    def num_axes(self):
        return 3

class FileTypeException(Exception):
    pass

class EncodingFailedException(Exception):
    pass

def format_array(arr):
    if arr is None:
        return None

    if not isinstance(arr, np.ndarray):
        arr = np.array(arr)
    if arr.ndim == 1:
        arr = arr.reshape((len(arr) // 3, 3))
    return arr

def encode(
    points, faces=None, 
    quantization_bits=14, compression_level=1, 
    quantization_range=-1, quantization_origin=None, 
    create_metadata=False
) -> bytes:
    """
    bytes encode(
        points, faces=None, 
        quantization_bits=14, compression_level=1, 
        quantization_range=-1, quantization_origin=None, 
        create_metadata=False
    )

    Encode a list or numpy array of points/vertices (float) and faces 
    (unsigned int) to a draco buffer. If faces is None, then a point
    cloud file will be generated, otherwise a mesh file.

    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the 
        bounding cube for the mesh. By default it is the range 
        of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. 
        By default it is a point where each coordinate is the minimum of 
        that coordinate among the input vertices.
    """
    assert 0 <= compression_level <= 10, "Compression level must be in range [0,10]"
    assert 0 <= quantization_bits <= 31, "Compression level must be in range [0,31]"

    points = format_array(points)
    faces = format_array(faces)

    integer_positions = np.issubdtype(points.dtype, np.integer)

    cdef np.ndarray[float, ndim=1] qorigin = np.zeros((3,), dtype=np.float32)
    cdef float[:] quant_origin = qorigin

    if quantization_origin is not None:
        qorigin[:] = quantization_origin[:]
    else:
        qorigin[:] = np.min(points, axis=0)

    cdef vector[float] pointsview = points.reshape((points.size,))
    cdef vector[uint32_t] facesview

    if faces is None:
        encoded = DracoPy.encode_point_cloud(
            pointsview, quantization_bits, compression_level, 
            quantization_range, <float*>&quant_origin[0], 
            create_metadata, integer_positions
        )
    else:
        facesview = faces.reshape((faces.size,))
        encoded = DracoPy.encode_mesh(
            pointsview, facesview, 
            quantization_bits, compression_level, 
            quantization_range, &quant_origin[0], 
            create_metadata, integer_positions
        )

    if encoded.encode_status == DracoPy.encoding_status.successful_encoding:
        return bytes(encoded.buffer)
    elif encoded.encode_status == DracoPy.encoding_status.failed_during_encoding:
        raise EncodingFailedException('Invalid mesh')

def raise_decoding_error(decoding_status):
    if decoding_status == DracoPy.decoding_status.not_draco_encoded:
        raise FileTypeException('Input mesh is not draco encoded')
    elif decoding_status == DracoPy.decoding_status.failed_during_decoding:
        raise TypeError('Failed to decode input mesh. Data might be corrupted')
    elif decoding_status == DracoPy.decoding_status.no_position_attribute:
        raise ValueError('DracoPy only supports meshes with position attributes')

def decode(bytes buffer) -> Union[DracoMesh, DracoPointCloud]:
    """
    (DracoMesh|DracoPointCloud) decode(bytes buffer)

    Decodes a binary draco file into either a DracoPointCloud
    or a DracoMesh.
    """
    mesh_struct = DracoPy.decode_buffer(buffer, len(buffer))
    if mesh_struct.decode_status != DracoPy.decoding_status.successful:
        raise_decoding_error(mesh_struct.decode_status)

    if len(mesh_struct.faces) > 0:
        return DracoMesh(mesh_struct)
    return DracoPointCloud(mesh_struct)

# FOR BACKWARDS COMPATIBILITY

def encode_mesh_to_buffer(*args, **kwargs) -> bytes:
    """Provided for backwards compatibility. Use encode."""
    return encode(*args, **kwargs)

def encode_point_cloud_to_buffer(
    points, quantization_bits=14, compression_level=1, 
    quantization_range=-1, quantization_origin=None, 
    create_metadata=False
) -> bytes:
    """Provided for backwards compatibility. Use encode."""
    return encode(
        points=points, 
        faces=None, 
        quantization_bits=quantization_bits, 
        compression_level=compression_level,
        quantization_range=quantization_range, 
        quantization_origin=quantization_origin, 
        create_metadata=create_metadata,
    )
 
def decode_buffer_to_mesh(buffer) -> Union[DracoMesh, DracoPointCloud]:
    """Provided for backwards compatibility. Use decode."""
    return decode(buffer)

def decode_buffer_to_point_cloud(buffer) -> Union[DracoMesh, DracoPointCloud]:
    """Provided for backwards compatibility. Use decode."""
    return cast(decode(buffer), DracoPointCloud)



