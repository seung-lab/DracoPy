# distutils: language = c++

from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport DracoPy
import struct
from math import floor
from libc.string cimport memcmp

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
        return self.data_struct['points']

class DracoMesh(DracoPointCloud):
    @property
    def faces(self):
        return self.data_struct['faces']

    @property
    def normals(self):
        return self.data_struct['normals']

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

def encode_mesh_to_buffer(
    points, faces, 
    quantization_bits=14, compression_level=1, 
    quantization_range=-1, quantization_origin=None, 
    create_metadata=False
):
    """
    Encode a list or numpy array of points/vertices (float) and faces (unsigned int) to a draco buffer.
    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the bounding cube for the mesh.
    By default it is the range of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. By default it is
    a point where each coordinate is the minimum of that coordinate among the input vertices.
    """
    if not isinstance(points, np.ndarray):
        points = np.array(points) 

    integer_positions = np.issubdtype(points.dtype, np.integer)

    cdef np.ndarray[float, ndim=1] qorigin = np.zeros((3,), dtype=np.float32)
    cdef float[:] quant_origin = qorigin

    if quantization_origin is not None:
        qorigin[:] = quantization_origin[:]
    else:
        qorigin[:] = np.min(points, axis=0)

    encoded_mesh = DracoPy.encode_mesh(
        points, faces, quantization_bits, compression_level, 
        quantization_range, &quant_origin[0], create_metadata,
        integer_positions
    )

    if encoded_mesh.encode_status == DracoPy.encoding_status.successful_encoding:
        return bytes(encoded_mesh.buffer)
    elif encoded_mesh.encode_status == DracoPy.encoding_status.failed_during_encoding:
        raise EncodingFailedException('Invalid mesh')


def encode_point_cloud_to_buffer(
    points, quantization_bits=14, 
    compression_level=1, quantization_range=-1, 
    quantization_origin=None, create_metadata=False
):
    """
    Encode a list or numpy array of points/vertices (float) to a draco buffer.
    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the bounding cube for the mesh.
    By default it is the range of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. By default it is
    a point where each coordinate is the minimum of that coordinate among the input vertices.
    """

    cdef np.ndarray[float, ndim=1] qorigin = np.zeros((3,), dtype=np.float32)
    cdef float[:] quant_origin = qorigin

    if quantization_origin is not None:
        qorigin[:] = quantization_origin[:]
    else:
        qorigin[:] = np.min(points, axis=0)

    encoded_point_cloud = DracoPy.encode_point_cloud(
        points, quantization_bits, compression_level, 
        quantization_range, <float*>&quant_origin[0], create_metadata
    )

    if encoded_point_cloud.encode_status == DracoPy.encoding_status.successful_encoding:
        return bytes(encoded_point_cloud.buffer)
    elif encoded_point_cloud.encode_status == DracoPy.encoding_status.failed_during_encoding:
        raise EncodingFailedException('Invalid mesh')

def raise_decoding_error(decoding_status):
    if decoding_status == DracoPy.decoding_status.not_draco_encoded:
        raise FileTypeException('Input mesh is not draco encoded')
    elif decoding_status == DracoPy.decoding_status.failed_during_decoding:
        raise TypeError('Failed to decode input mesh. Data might be corrupted')
    elif decoding_status == DracoPy.decoding_status.no_position_attribute:
        raise ValueError('DracoPy only supports meshes with position attributes')

def decode_buffer_to_mesh(buffer):
    mesh_struct = DracoPy.decode_buffer(buffer, len(buffer))
    if mesh_struct.decode_status == DracoPy.decoding_status.successful:
        return DracoMesh(mesh_struct)
    else:
        raise_decoding_error(mesh_struct.decode_status)

def decode_point_cloud_buffer(buffer):
    point_cloud_struct = DracoPy.decode_buffer_to_point_cloud(buffer, len(buffer))
    if point_cloud_struct.decode_status == DracoPy.decoding_status.successful:
        return DracoPointCloud(point_cloud_struct)
    else:
        raise_decoding_error(point_cloud_struct.decode_status)