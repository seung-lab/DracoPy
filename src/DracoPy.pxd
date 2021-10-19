from libcpp.vector cimport vector
from libcpp.string cimport string
from libc.stdint cimport uint32_t
from libcpp cimport bool


cdef extern from "DracoPy.h" namespace "DracoFunctions":

    cdef enum decoding_status:
        successful, not_draco_encoded, no_position_attribute, 
        failed_during_decoding

    cdef enum encoding_status:
        successful_encoding, failed_during_encoding

    cdef enum endian_order:
        little_endian
        big_endian
        unknown_endian

    cdef struct PointCloudObject:
        vector[float] points

        # Encoding options
        bool encoding_options_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status

        string binary_metadata

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

        string binary_metadata

    cdef struct EncodedObject:
        vector[unsigned char] buffer
        encoding_status encode_status

    endian_order get_endian_order() except +

    MeshObject decode_buffer(const char *buffer, size_t buffer_len) except +

    PointCloudObject decode_buffer_to_point_cloud(const char *buffer, size_t buffer_len) except +

    EncodedObject encode_mesh(vector[float] points, vector[uint32_t] faces, int quantization_bits,
        int compression_level, float quantization_range, const float *quantization_origin, bool create_metadata) except +
    
    EncodedObject encode_point_cloud(vector[float] points, int quantization_bits,
        int compression_level, float quantization_range, const float *quantization_origin, bool create_metadata) except +
