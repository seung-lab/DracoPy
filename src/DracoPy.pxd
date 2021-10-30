from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.unordered_map cimport unordered_map
from libc.stdint cimport uint32_t, uint8_t
from libcpp cimport bool

cdef extern from "DracoPy.h" namespace "draco":
    cdef enum DataType:
        DT_INVALID
        DT_INT8
        DT_UINT8
        DT_INT16
        DT_UINT16
        DT_INT32
        DT_UINT32
        DT_INT64
        DT_UINT64
        DT_FLOAT32
        DT_FLOAT64
        DT_BOOL
        DT_TYPES_COUNT


cdef extern from "DracoPy.h" namespace "DracoFunctions":

    cdef enum decoding_status:
        successful, not_draco_encoded, no_position_attribute, 
        failed_during_decoding

    cdef enum encoding_status:
        successful_encoding, failed_during_encoding
    
    cdef struct MetadataObject:
        unordered_map[string, string] entries
        unordered_map[string, uint32_t] sub_metadata_ids

    cdef struct PointAttributeObject:
        unordered_map[uint32_t, string] data
        DataType datatype
        uint32_t dimension
        uint32_t metadata_id
    
    cdef struct GeometryMetadataObject:
        uint32_t metadata_id  # reference to metadata object
        vector[PointAttributeObject] generic_attributes

    cdef struct PointCloudObject:
        vector[float] points

        # Encoding options
        bool encoding_options_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status

        GeometryMetadataObject geometry_metadata
        vector[MetadataObject] metadatas

    cdef struct MeshObject:
        vector[float] points
        vector[unsigned int] faces

        # TODO: add support for normals, which are not currently supported.
        vector[float] normals

        # Encoding options
        bool encoding_options_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status

        GeometryMetadataObject geometry_metadata
        vector[MetadataObject] metadatas

    cdef struct EncodedObject:
        vector[unsigned char] buffer
        encoding_status encode_status

    cdef cppclass MetadataReader:
        MetadataReader(const string& s) except +
        uint32_t read_uint() except +
        string read_bytes() except +

    cdef cppclass MetadataWriter:
        MetadataWriter() except +
        void write_uint(const uint32_t& value) except +
        void write_bytes_from_str(const string& value) except +
        void write_bytes_from_vec(const vector[uint8_t]& value) except +
        string get() except +

    MeshObject decode_buffer(const char *buffer, size_t buffer_len) except +

    PointCloudObject decode_buffer_to_point_cloud(const char *buffer, size_t buffer_len) except +

    EncodedObject encode_mesh(
            vector[float] points,
            vector[uint32_t] faces,
            vector[MetadataObject] metadatas,
            GeometryMetadataObject geometry_metadata,
            int quantization_bits, int compression_level,
            float quantization_range, const float *quantization_origin,
            bool create_metadata) except +
    
    EncodedObject encode_point_cloud(
            vector[float] points, PointCloudObject pco,
            int quantization_bits, int compression_level,
            float quantization_range, const float *quantization_origin,
            bool create_metadata) except +
