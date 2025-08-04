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

cimport numpy as cnp
cnp.import_array()

import numpy as np

class DracoPointCloud:
    def __init__(self, data_struct):
        self.data_struct = data_struct
        if data_struct['encoding_options_set']:
            self.encoding_options = EncodingOptions(data_struct['quantization_bits'],
                data_struct['quantization_range'], data_struct['quantization_origin'])
        else:
            self.encoding_options = None

        self._attributes = []
        
        attributes_list = self.data_struct['attributes']
        
        if len(attributes_list) > 0:    
            for attr in attributes_list:
                attr_info = {
                    'unique_id': attr.get('unique_id', 0),
                    'num_components': attr.get('num_components', 0),
                    'data_type': attr.get('data_type', 0),
                    'attribute_type': attr.get('attribute_type', 0),
                    'data': None
                }
                float_data = attr.get('float_data', [])
                uint_data = attr.get('uint_data', [])
                byte_data = attr.get('byte_data', [])

                # Get the appropriate data array based on data type
                data_array = None
                if len(float_data) > 0:
                    data_array = np.array(float_data, dtype=np.float32)
                elif len(uint_data) > 0:
                    data_array = np.array(uint_data, dtype=np.uint32)
                elif len(byte_data) > 0:
                    data_array = np.array(byte_data, dtype=np.uint8)

                if data_array is not None:
                    attr_info['data'] = data_array.reshape((-1, attr_info['num_components']))
                else:
                    attr_info['data'] = None

                self._attributes.append(attr_info)

    def get_encoded_coordinate(self, value, axis):
        if self.encoding_options is not None:
            return self.encoding_options.get_encoded_coordinate(value, axis)

    def get_encoded_point(self, point):
        if self.encoding_options is not None:
            return self.encoding_options.get_encoded_point(point)

    @property
    def num_axes(self):
        return 3

    def get_attribute_by_type(self, attribute_type):
        for attr in self.attributes:
            if attr['attribute_type'] == attribute_type:
                return attr
        return None

    def get_attribute_by_unique_id(self, unique_id):
        for attr in self.attributes:
            if attr['unique_id'] == unique_id:
                return attr
        return None

    @property
    def attributes(self):
        return self._attributes

    @property
    def points(self):
        position_attr = self.get_attribute_by_type(0)  # POSITION=0
        if position_attr and position_attr['data'] is not None:
            return position_attr['data']
        return None

    @property
    def colors(self):
        color_attr = self.get_attribute_by_type(2)  # COLOR=2
        if color_attr and color_attr['data'] is not None:
            return color_attr['data']
        return None

class DracoMesh(DracoPointCloud):
    @property
    def faces(self):
        faces_ = self.data_struct['faces']
        N = len(faces_) // 3
        return np.array(faces_, dtype=np.uint32).reshape((N, 3))

    @property
    def normals(self):
        normal_attr = self.get_attribute_by_type(1)  # NORMAL = 1
        if normal_attr and normal_attr['data'] is not None:
            return normal_attr['data']
        return None

    @property
    def tex_coord(self):
        tex_attr = self.get_attribute_by_type(3)  # TEX_COORD = 3
        if tex_attr and tex_attr['data'] is not None:
            return tex_attr['data']
        return None

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

def format_array(arr, col=3):
    if arr is None:
        return None

    if not isinstance(arr, np.ndarray):
        arr = np.array(arr)
    if arr.ndim == 1:
        arr = arr.reshape((len(arr) // col, col))
    return arr

def encode(
    points, faces=None,
    quantization_bits=14, compression_level=1,
    quantization_range=-1, quantization_origin=None,
    create_metadata=False, preserve_order=False,
    colors=None, tex_coord=None, normals=None,
    tangents=None, joints=None, weights=None
) -> bytes:
    """
    bytes encode(
        points, faces=None,
        quantization_bits=11, compression_level=1,
        quantization_range=-1, quantization_origin=None,
        create_metadata=False, preserve_order=False,
        colors=None, tex_coord=None, normals=None,
        tangents=None, joints=None, weights=None
    )

    Encode a list or numpy array of points/vertices (float) and faces
    (unsigned int) to a draco buffer. If faces is None, then a point
    cloud file will be generated, otherwise a mesh file.

    Quantization bits should be an integer between 1 and 30
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the
        bounding cube for the mesh. By default it is the range
        of the dimension of the input vertices with greatest range.
        Set a negative value to use the default behavior.
    Quantization_origin is the point in space where the bounding box begins.
        By default it is a point where each coordinate is the minimum of
        that coordinate among the input vertices.
    Preserve_order controls whether the order of points / faces should be
        preserved after compression. Setting it to True will reduce compression
        ratio (greatly) but guarantees the result points / faces are in same
        order as the input.
    Colors is a numpy array of colors (uint8) with shape (N, K). N is the number of
        vertices. K must be >= 1. Use None if mesh does not have colors
    Tex coord is a numpy array of texture coordinates (float) with shape (N, 2). N is the number of
        vertices. Use None if mesh does not have texture coordinates.
    Normals is a numpy array of normal vectors (float) with shape (N, 3). N is the number of
        vertices. Use None if mesh does not have normal vectors.
    Tangents is a numpy array of tangent vectors (float) with shape (N, 3) or (N, 4). N is the number of
        vertices. Use None if mesh does not have tangent vectors.
    Joints is a numpy array of joint indices (uint16) with shape (N, K). N is the number of
        vertices. K is the number of joint influences per vertex. Use None if mesh does not have skeletal data.
    Weights is a numpy array of joint weights (float) with shape (N, K). N is the number of
        vertices. K is the number of joint influences per vertex. Use None if mesh does not have skeletal data.
    """
    assert 0 <= compression_level <= 10, "Compression level must be in range [0, 10]"

    # @zeruniverse Draco supports quantization_bits 1 to 30, see following link:
    # https://github.com/google/draco/blob/master/src/draco/attributes/attribute_quantization_transform.cc#L107
    assert 1 <= quantization_bits <= 30, "Quantization bits must be in range [1, 30]"

    points = format_array(points)
    faces = format_array(faces)
    colors = format_array(colors)
    tex_coord = format_array(tex_coord, col=2)
    normals = format_array(normals, col=3)
    tangents = format_array(tangents)
    joints = format_array(joints)
    weights = format_array(weights)

    integer_mark = 0

    if np.issubdtype(points.dtype, np.signedinteger):
        integer_mark = 1
    elif np.issubdtype(points.dtype, np.unsignedinteger):
        integer_mark = 2

    cdef cnp.ndarray[float, ndim=1] qorigin = np.zeros((3,), dtype=np.float32)
    cdef float[:] quant_origin = qorigin

    if quantization_origin is not None:
        qorigin[:] = quantization_origin[:]
    else:
        qorigin[:] = np.min(points, axis=0)

    cdef vector[float] pointsview = points.reshape((points.size,))
    cdef vector[uint32_t] facesview
    cdef vector[uint8_t] colorsview
    cdef vector[float] texcoordview
    cdef vector[float] normalsview
    cdef vector[float] tangentsview
    cdef vector[unsigned short] jointsview
    cdef vector[float] weightsview

    colors_channel = 0
    if colors is not None:
        assert np.issubdtype(colors.dtype, np.uint8), "Colors must be uint8"
        assert len(colors.shape) == 2, "Colors must be 2D"
        colors_channel = colors.shape[1]
        assert 1 <= colors_channel <= 127, "Number of color channels must be in range [1, 127]"
        colorsview = colors.reshape((colors.size,))

    tex_coord_channel = 0
    if tex_coord is not None:
        assert np.issubdtype(tex_coord.dtype, float), "Tex coord must be float"
        assert len(tex_coord.shape) == 2, "Tex coord must be 2D"
        tex_coord_channel = tex_coord.shape[1]
        assert 1 <= tex_coord_channel <= 127, "Number of tex coord channels must be in range [1, 127]"
        texcoordview = tex_coord.reshape((tex_coord.size,))


    has_normals = 0
    if normals is not None:
        assert np.issubdtype(normals.dtype, float), "Normals must be float"
        assert normals.shape[1] == 3, "Normals must have 3 components"
        has_normals = 1
        normalsview = normals.reshape((normals.size,))

    tangent_channel = 0
    if tangents is not None:
        assert np.issubdtype(tangents.dtype, np.floating), "Tangents must be float"
        assert len(tangents.shape) == 2, "Tangents must be 2D"
        tangent_channel = tangents.shape[1]
        assert tangent_channel == 3 or tangent_channel == 4, "Tangents must have 3 or 4 components"
        tangentsview = tangents.reshape((tangents.size,))

    joint_channel = 0
    if joints is not None:
        assert np.issubdtype(joints.dtype, np.integer), "Joints must be integer"
        assert len(joints.shape) == 2, "Joints must be 2D"
        joint_channel = joints.shape[1]
        assert 1 <= joint_channel <= 16, "Number of joint channels must be in range [1, 16]"
        joints = joints.astype(np.uint16)
        jointsview = joints.reshape((joints.size,))

    weight_channel = 0
    if weights is not None:
        assert np.issubdtype(weights.dtype, np.floating), "Weights must be float"
        assert len(weights.shape) == 2, "Weights must be 2D"
        weight_channel = weights.shape[1]
        assert 1 <= weight_channel <= 16, "Number of weight channels must be in range [1, 16]"
        weightsview = weights.reshape((weights.size,))

    if faces is None:
        encoded = DracoPy.encode_point_cloud(
            pointsview, quantization_bits, compression_level,
            quantization_range, <float*>&quant_origin[0],
            preserve_order, create_metadata, integer_mark,
            colorsview, colors_channel
        )
    else:
        facesview = faces.reshape((faces.size,))
        encoded = DracoPy.encode_mesh(
            pointsview, facesview,
            quantization_bits, compression_level,
            quantization_range, &quant_origin[0],
            preserve_order, create_metadata, integer_mark,
            colorsview, colors_channel, texcoordview, tex_coord_channel,
            normalsview, has_normals,
            tangentsview, tangent_channel,
            jointsview, joint_channel,
            weightsview, weight_channel
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
        preserve_order=False
    )

def decode_buffer_to_mesh(buffer) -> Union[DracoMesh, DracoPointCloud]:
    """Provided for backwards compatibility. Use decode."""
    return decode(buffer)

def decode_buffer_to_point_cloud(buffer) -> Union[DracoMesh, DracoPointCloud]:
    """Provided for backwards compatibility. Use decode."""
    return cast(decode(buffer), DracoPointCloud)
