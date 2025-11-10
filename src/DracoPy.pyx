# distutils: language = c++
from typing import Union, cast

from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport DracoPy
import struct
from math import floor
from libcpp.string cimport string
from libc.string cimport memcmp
from libc.stdint cimport (
  int8_t, int16_t, int32_t, int64_t,
  uint8_t, uint16_t, uint32_t, uint64_t,
)

cimport numpy as cnp
cnp.import_array()

import numpy as np
from enum import IntEnum

class DataType(IntEnum):
    DT_INVALID = 0
    DT_INT8 = 1
    DT_UINT8 = 2
    DT_INT16 = 3
    DT_UINT16 = 4
    DT_INT32 = 5
    DT_UINT32 = 6
    DT_INT64 = 7
    DT_UINT64 = 8
    DT_FLOAT32 = 9
    DT_FLOAT64 = 10
    DT_BOOL = 11

class AttributeType(IntEnum):
    INVALID = -1
    POSITION = 0
    NORMAL = 1
    COLOR = 2
    TEX_COORD = 3
    GENERIC = 4

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

                name = attr.get('name', None)
                attr_info['name'] = name.decode('utf-8') if name else None

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

    def get_attribute_by_name(self, name):
        for attr in self.attributes:
            if attr['name'] == name:
                return attr
        return None

    @property
    def attributes(self):
        return self._attributes

    @property
    def points(self):
        position_attr = self.get_attribute_by_type(AttributeType.POSITION)  # POSITION=0
        if position_attr and position_attr['data'] is not None:
            return position_attr['data']
        return None

    @property
    def colors(self):
        color_attr = self.get_attribute_by_type(AttributeType.COLOR)  # COLOR=2
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
        normal_attr = self.get_attribute_by_type(AttributeType.NORMAL)  # NORMAL = 1
        if normal_attr and normal_attr['data'] is not None:
            return normal_attr['data']
        return None

    @property
    def tex_coord(self):
        tex_attr = self.get_attribute_by_type(AttributeType.TEX_COORD)  # TEX_COORD = 3
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
    generic_attributes=None
) -> bytes:
    """
    bytes encode(
        points, faces=None,
        quantization_bits=11, compression_level=1,
        quantization_range=-1, quantization_origin=None,
        create_metadata=False, preserve_order=False,
        colors=None, tex_coord=None, normals=None,
        generic_attributes=None
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
    Generic_attributes is a dictionary of additional attributes to encode, where:
       - Keys: non-negative integer unique_ids or strings
         - When the key is an integer, it is used as the attribute's unique_id.
         - When it is a string, it is stored in the attribute's metadata entry as "name".
           Use the `get_attribute_by_name()` method to retrieve attributes by name.
       - Values: numpy arrays with shape (N, K) where:
         - N = number of vertices in the mesh
         - K = number of components per attribute
       - Supported data types: float, uint8, uint16, uint32
       - Use None if there are no generic attributes to encode.

        @example
        ```python
        # Example with additional vertex tangents, joints, and weights
        generic_attributes = {
            0: vertex_tangents,    # shape (1000, 3) for tangents, unique_id is 0
            1: vertex_joints,      # shape (1000, 4) for joints, unique_id is 1
            4: vertex_weights      # shape (1000, 4) for weights, unique_id is 4
        }

        # Use string keys to name attributes
        generic_attributes = {
            "tangents": vertex_tangents,
            "joints": vertex_joints,
            "weights": vertex_weights
        }
        ```
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


    # Process generic attributes from generic_attributes
    cdef vector[int8_t] unique_ids
    cdef vector[vector[float]] attr_float_data
    cdef vector[vector[uint8_t]] attr_uint8_data
    cdef vector[vector[uint16_t]] attr_uint16_data
    cdef vector[vector[uint32_t]] attr_uint32_data
    cdef vector[int] attr_data_types  # 0=float, 1=uint8, 2=uint16, 3=uint32
    cdef vector[int] attr_num_components
    cdef vector[string] attr_names

    if generic_attributes:
        for id_or_name, attr_data in generic_attributes.items():
            if type(id_or_name) not in (int, str):
                raise ValueError(f"Generic attribute keys must be integers or strings")
            if type(id_or_name) == int and id_or_name < 0:
                raise ValueError(f"Generic attribute IDs must be positive integers")

            if attr_data is None:
                continue
                
            # Format the attribute data
            attr_array = format_array(attr_data)
            if attr_array is None:
                continue
                
            # Validate attribute array
            if len(attr_array.shape) != 2:
                raise ValueError(f"Attribute '{id_or_name}' must be 2D array")
            if attr_array.shape[0] != points.shape[0]:
                raise ValueError(f"Attribute '{id_or_name}' must have same number of vertices as points")
            
            if type(id_or_name) == int:
                unique_ids.push_back(id_or_name)
                attr_names.push_back("")
            else:
                unique_ids.push_back(-1)
                attr_names.push_back(id_or_name.encode('utf-8'))

            # Store attribute info
            attr_num_components.push_back(attr_array.shape[1])
            
            # Handle different data types
            if np.issubdtype(attr_array.dtype, np.floating):
                attr_data_types.push_back(DataType.DT_FLOAT32)  # 9, float
                attr_array = attr_array.astype(np.float32)
                float_view = attr_array.reshape((attr_array.size,))
                attr_float_data.push_back(float_view)
                # Add empty vectors for other types
                attr_uint8_data.push_back(vector[uint8_t]())
                attr_uint16_data.push_back(vector[uint16_t]())
                attr_uint32_data.push_back(vector[uint32_t]())
            elif attr_array.dtype == np.uint8:
                attr_data_types.push_back(DataType.DT_UINT8)  # 2, uint8
                uint8_view = attr_array.reshape((attr_array.size,))
                attr_uint8_data.push_back(uint8_view)
                # Add empty vectors for other types
                attr_float_data.push_back(vector[float]())
                attr_uint16_data.push_back(vector[uint16_t]())
                attr_uint32_data.push_back(vector[uint32_t]())
            elif attr_array.dtype == np.uint16:
                attr_data_types.push_back(DataType.DT_UINT16)  # 4, uint16
                uint16_view = attr_array.reshape((attr_array.size,))
                attr_uint16_data.push_back(uint16_view)
                # Add empty vectors for other types
                attr_float_data.push_back(vector[float]())
                attr_uint8_data.push_back(vector[uint8_t]())
                attr_uint32_data.push_back(vector[uint32_t]())
            elif attr_array.dtype == np.uint32:
                attr_data_types.push_back(DataType.DT_UINT32)  # 6, uint32
                uint32_view = attr_array.reshape((attr_array.size,))
                attr_uint32_data.push_back(uint32_view)
                # Add empty vectors for other types
                attr_float_data.push_back(vector[float]())
                attr_uint8_data.push_back(vector[uint8_t]())
                attr_uint16_data.push_back(vector[uint16_t]())
            else:
                raise ValueError(f"Unsupported data type for attribute '{id_or_name}': {attr_array.dtype}")

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

    if faces is None:
        encoded = DracoPy.encode_point_cloud(
            pointsview, quantization_bits, compression_level,
            quantization_range, <float*>&quant_origin[0],
            preserve_order, create_metadata, integer_mark,
            colorsview, colors_channel,
            unique_ids, attr_float_data, attr_uint8_data,
            attr_uint16_data, attr_uint32_data,
            attr_data_types, attr_num_components,
            attr_names
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
            unique_ids, attr_float_data, attr_uint8_data,
            attr_uint16_data, attr_uint32_data,
            attr_data_types, attr_num_components,
            attr_names
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