# distutils: language = c++
import typing

from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport DracoPy
from math import floor
from libc.string cimport memcmp


class _Reader:
    def __init__(self, in_bytes: bytes):
        self._in_bytes = in_bytes
        self._pos = 0

    @staticmethod
    def __get_endian() -> str:
        endian_order = get_endian_order()
        if endian_order == DracoPy.endian_order.little_endian:
            return "little"
        elif endian_order == DracoPy.endian_order.big_endian:
            return "big"
        else:
            raise Exception("unknown endian order")

    def __read_bytes(self, count: int) -> bytes:
        end = self._pos + count
        result_bytes = self._in_bytes[self._pos:end]
        self._pos = end
        return result_bytes

    def read_uint(self) -> int:
        value = int.from_bytes(self.__read_bytes(4),
                               byteorder=self.__get_endian(), signed=False)
        return value

    def read_str(self) -> str:
        count = self.read_uint()
        return self.__read_bytes(count).decode()

    def read_bytes(self) -> bytes:
        count = self.read_uint()
        return self.__read_bytes(count)

    def is_end(self) -> bool:
        return len(self._in_bytes) == self._pos


class MetadataObject:
    def __init__(self, entries: typing.Dict[str, bytes] = None,
                 sub_metadatas: typing.Dict[str, 'MetadataObject'] = None):
        self.entries = entries if entries else {}
        self.sub_metadatas = sub_metadatas if sub_metadatas else {}


class GeometryMetadataObject(MetadataObject):
    def __init__(self, entries: typing.Dict[str, bytes] = None,
                 sub_metadatas: typing.Dict[str, 'MetadataObject'] = None,
                 attribute_metadatas: typing.List['MetadataObject'] = None):
        super().__init__(entries, sub_metadatas)
        self.attribute_metadatas = attribute_metadatas if attribute_metadatas \
            else []


def _parse_binary_metadata(binary_metadata: bytes) -> MetadataObject:
    reader = _Reader(binary_metadata)
    geometry_metadata = GeometryMetadataObject()
    to_parse_metadatas = [geometry_metadata]
    # parse attribute metadatas
    attribute_metadatas_len = reader.read_uint()
    for _ in range(attribute_metadatas_len):
        attribute_metadata = MetadataObject()
        geometry_metadata.attribute_metadatas.append(attribute_metadata)
        to_parse_metadatas.append(attribute_metadata)
    # parse metadatas level by level
    while to_parse_metadatas:
        to_parse_metadata_next = []
        for metadata in to_parse_metadatas:
            # parse entries
            attr_len = reader.read_uint()
            for _ in range(attr_len):
                name = reader.read_str()
                value = reader.read_bytes()
                metadata.entries[name] = value
            sub_metadatas_len = reader.read_uint()
            for _ in range(sub_metadatas_len):
                name = reader.read_str()
                sub_metadata = MetadataObject()
                metadata.sub_metadatas[name] = sub_metadata
                to_parse_metadata_next.append(sub_metadata)
        to_parse_metadatas = to_parse_metadata_next
    if not reader.is_end():
        raise Exception("not read bytes detected")
    return geometry_metadata


class DracoPointCloud(object):
    def __init__(self, data_struct):
        self.data_struct = data_struct
        if data_struct['encoding_options_set']:
            self.encoding_options = EncodingOptions(data_struct['quantization_bits'],
                data_struct['quantization_range'], data_struct['quantization_origin'])
        else:
            self.encoding_options = None
        self.metadata = _parse_binary_metadata(data_struct["binary_metadata"])

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

def encode_mesh_to_buffer(points, faces, quantization_bits=14, compression_level=1, quantization_range=-1, quantization_origin=None, create_metadata=False):
    """
    Encode a list or numpy array of points/vertices (float) and faces (unsigned int) to a draco buffer.
    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the bounding cube for the mesh.
    By default it is the range of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. By default it is
    a point where each coordinate is the minimum of that coordinate among the input vertices.
    """
    cdef float* quant_origin = NULL
    try:
        num_dims = 3
        if quantization_origin is not None:
            quant_origin = <float *>PyMem_Malloc(sizeof(float) * num_dims)
            for dim in range(num_dims):
                quant_origin[dim] = quantization_origin[dim]
        encoded_mesh = DracoPy.encode_mesh(points, faces, quantization_bits, compression_level, quantization_range, quant_origin, create_metadata)
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        if encoded_mesh.encode_status == DracoPy.encoding_status.successful_encoding:
            return bytes(encoded_mesh.buffer)
        elif encoded_mesh.encode_status == DracoPy.encoding_status.failed_during_encoding:
            raise EncodingFailedException('Invalid mesh')
    except EncodingFailedException:
        raise EncodingFailedException('Invalid mesh')
    except:
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        raise ValueError("Input invalid")

def encode_point_cloud_to_buffer(points, quantization_bits=14, compression_level=1, quantization_range=-1, quantization_origin=None, create_metadata=False):
    """
    Encode a list or numpy array of points/vertices (float) to a draco buffer.
    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the bounding cube for the mesh.
    By default it is the range of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. By default it is
    a point where each coordinate is the minimum of that coordinate among the input vertices.
    """
    cdef float* quant_origin = NULL
    try:
        num_dims = 3
        if quantization_origin is not None:
            quant_origin = <float *>PyMem_Malloc(sizeof(float) * num_dims)
            for dim in range(num_dims):
                quant_origin[dim] = quantization_origin[dim]
        encoded_point_cloud = DracoPy.encode_point_cloud(points, quantization_bits, compression_level, quantization_range, quant_origin, create_metadata)
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        if encoded_point_cloud.encode_status == DracoPy.encoding_status.successful_encoding:
            return bytes(encoded_point_cloud.buffer)
        elif encoded_point_cloud.encode_status == DracoPy.encoding_status.failed_during_encoding:
            raise EncodingFailedException('Invalid point cloud')
    except EncodingFailedException:
        raise EncodingFailedException('Invalid point cloud')
    except:
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        raise ValueError("Input invalid")

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