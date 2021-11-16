# distutils: language = c++
from typing import Dict, List, Optional

from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport DracoPy
from math import floor


class DracoPointCloud(object):
    def __init__(self, data_struct):
        self.data_struct = data_struct
        if data_struct['encoding_options_set']:
            self.encoding_options = EncodingOptions(data_struct['quantization_bits'],
                data_struct['quantization_range'], data_struct['quantization_origin'])
        else:
            self.encoding_options = None
        # self.metadata = decode_metadata(data_struct["binary_metadata"])

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
    def points(self) -> List[float]:
        return self.data_struct['points']

    @property
    def geometry_metadata(self) -> Optional[Dict]:
        return self.data_struct['geometry_metadata'] if self.metadatas else None

    @property
    def metadatas(self) -> List[Dict]:
        return self.data_struct['metadatas']


class DracoMesh(DracoPointCloud):
    @property
    def faces(self) -> List[int]:
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

def create_empty_geometry_metadata() -> dict:
    return {
        "metadata_id": 0,
        "generic_attributes": [],
    }

def encode_mesh_to_buffer(points, faces,
                          quantization_bits=14,
                          compression_level=1,
                          quantization_range=-1,
                          quantization_origin=None,
                          create_metadata=False,
                          metadatas = None,
                          geometry_metadata = None,
                          ):
    return encode_to_buffer(points, faces, quantization_bits, compression_level,
                            quantization_range, quantization_origin,
                            create_metadata, metadatas, geometry_metadata)


def encode_point_cloud_to_buffer(points,
                                 quantization_bits=14,
                                 compression_level=1,
                                 quantization_range=-1,
                                 quantization_origin=None,
                                 create_metadata=False,
                                 metadatas = None,
                                 geometry_metadata = None,
                                 ):
    return encode_to_buffer(points, None, quantization_bits, compression_level,
                            quantization_range, quantization_origin,
                            create_metadata, metadatas, geometry_metadata)


def encode_to_buffer(points: List[float],
                     faces: List[int] = None,
                     quantization_bits=14,
                     compression_level=1,
                     quantization_range=-1,
                     quantization_origin=None,
                     create_metadata=False,
                     metadatas = None,
                     geometry_metadata = None,
                     ):
    """
    Encode a list or numpy array of points/vertices (float) to a draco buffer.
    :param List[float] points: vector of points coordination
    :param Optional[List[int]] faces: vector of points indexes
    (each triple means one face). If faces is None then point cloud
    will be encoded, otherwise mesh will be encoded
    :param int quantization_bits: integer between 0 and 31
    :param int compression_level: integer between 0 and 10
    :param float quantization_range: float representing the size
    of the bounding cube for the mesh. By default it is the range of
    the dimension of the input vertices with greatest range.
    :param quantization_origin: point in space where the bounding box begins.
    By default it is a point where each coordinate is the minimum
    of that coordinate among the input vertices.
    :param bool create_metadata: if True then it creates GeometryMetadata
    :param List[dict] metadatas: list of metadatas each of them containing
        "entries" - dictionary with strings (entry name) and binary data
                    related to that entry
        "sub_metadata_ids" - dictionary with strings (submetadata name) and
                             related submetadata index in the list 'metadatas'
    :param dict geometry_metadata: dict containing the following items:
        "metadata_id" - index in the list 'metadatas' related to that metadata
        "generic_attributes" - list of geometry attributes (dict) each of them contain:
            "data" - dictionary with point index (not pure points index)
                     from points list
            "datatype" - type of the data item (see DataType enum)
            "dimension" - integer that defines number of data items with type 'datatype'
                          are placed per point
            "metadata_id" - metadata index in 'metadatas'
    NOTE: all 'metadata_id' indexes have to exists in 'metadatas' fields
    :return bytes: encoded mesh
    """
    if faces is None and isinstance(geometry_metadata, dict) and \
            len(geometry_metadata["generic_attributes"]) > 0:
        raise RuntimeError("generic attributes encoding/decoding "
                           "is not supported for point cloud")
    if metadatas is None:
        metadatas = []
    if geometry_metadata is None:
        geometry_metadata = create_empty_geometry_metadata()
    cdef float* quant_origin = NULL
    try:
        num_dims = 3
        if quantization_origin is not None:
            quant_origin = <float *>PyMem_Malloc(sizeof(float) * num_dims)
            for dim in range(num_dims):
                quant_origin[dim] = quantization_origin[dim]
        if isinstance(faces, list):
            draco_encoded = DracoPy.encode_mesh(points,
                                                faces,
                                                metadatas,
                                                geometry_metadata,
                                                quantization_bits,
                                                compression_level,
                                                quantization_range,
                                                quant_origin,
                                                create_metadata)
        else:
            draco_encoded = DracoPy.encode_point_cloud(points,
                                                       metadatas,
                                                       geometry_metadata,
                                                       quantization_bits,
                                                       compression_level,
                                                       quantization_range,
                                                       quant_origin,
                                                       create_metadata)
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        if draco_encoded.encode_status == DracoPy.encoding_status.successful_encoding:
            return bytes(draco_encoded.buffer)
        elif draco_encoded.encode_status == DracoPy.encoding_status.failed_during_encoding:
            raise EncodingFailedException('Invalid draco structure')
    except EncodingFailedException:
        raise EncodingFailedException('Invalid draco structure')
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

def decode_buffer_to_mesh(buffer, deduplicate=False):
    """
    Decode buffer to mesh
    :param bytes buffer: encoded mesh
    :param bool deduplicate: run Draco deduplcation functions
    :return: mesh object
    """
    mesh_struct = DracoPy.decode_buffer(buffer, len(buffer), deduplicate)
    if mesh_struct.decode_status == DracoPy.decoding_status.successful:
        return DracoMesh(mesh_struct)
    else:
        raise_decoding_error(mesh_struct.decode_status)

def decode_point_cloud_buffer(buffer, deduplicate=False):
    """
    Decode buffer to point cloud
    :param bytes buffer: encoded point cloud
    :param bool deduplicate: run Draco deduplcation functions
    :return: point cloud object
    """
    point_cloud_struct = DracoPy.decode_buffer_to_point_cloud(buffer, len(buffer), deduplicate)
    if point_cloud_struct.decode_status == DracoPy.decoding_status.successful:
        return DracoPointCloud(point_cloud_struct)
    else:
        raise_decoding_error(point_cloud_struct.decode_status)