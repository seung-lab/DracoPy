#cython: language_level=3
from libcpp.vector cimport vector
from libc.stdint cimport uint8_t, uint32_t
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
        bool colors_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status
        vector[uint8_t] colors

    cdef struct MeshObject:
        vector[float] points
        vector[unsigned int] faces

        vector[float] normals
        vector[float] tex_coord

        # Encoding options
        bool encoding_options_set
        bool colors_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status
        vector[uint8_t] colors

    cdef struct EncodedObject:
        vector[unsigned char] buffer
        encoding_status encode_status

    MeshObject decode_buffer(const char *buffer, size_t buffer_len) except +

    EncodedObject encode_mesh(
        const vector[float] points,
        const vector[uint32_t] faces,
        const int quantization_bits,
        const int compression_level,
        const float quantization_range,
        const float *quantization_origin,
        const bool preserve_order,
        const bool create_metadata,
        const int integer_mark,
        const vector[uint8_t] colors,
        const uint8_t colors_channel,
        const vector[float] tex_coord,
        const uint8_t tex_coord_channel,
    ) except +

    EncodedObject encode_point_cloud(
        const vector[float] points,
        const int quantization_bits,
        const int compression_level,
        const float quantization_range,
        const const float *quantization_origin,
        const bool preserve_order,
        const bool create_metadata,
        const int integer_mark,
        const vector[uint8_t] colors,
        const uint8_t colors_channel
    ) except +
