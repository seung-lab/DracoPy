from libcpp.vector cimport vector
from libc.stdint cimport uint32_t
from libcpp cimport bool

cimport numpy
import numpy as np

cdef extern from "DracoPy.h" namespace "DracoFunctions":

    cdef enum decoding_status:
        successful, not_draco_encoded, no_position_attribute, 
        failed_during_decoding

    cdef enum encoding_status:
        successful_encoding, failed_during_encoding

    cdef struct PointCloudObject:
        vector[float] points

        # Encoding options
        bool encoding_options_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status

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

    cdef struct EncodedObject:
        vector[unsigned char] buffer
        encoding_status encode_status

    MeshObject decode_buffer(const char *buffer, size_t buffer_len) except +

    PointCloudObject decode_buffer_to_point_cloud(const char *buffer, size_t buffer_len) except +

    EncodedObject encode_mesh(
        const vector[float] points, 
        const vector[uint32_t] faces, 
        const int quantization_bits,
        const int compression_level, 
        const float quantization_range, 
        const float *quantization_origin, 
        const bool create_metadata,
        const bool integer_positions
    ) except +
    
    EncodedObject encode_point_cloud(
        const vector[float] points, 
        const int quantization_bits,
        const int compression_level, 
        const float quantization_range, 
        const const float *quantization_origin, 
        const bool create_metadata,
        const bool integer_positions
    ) except +
