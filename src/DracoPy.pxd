#cython: language_level=3
from libcpp.vector cimport vector
from libc.stdint cimport uint8_t, uint16_t, uint32_t
from libcpp cimport bool

cimport numpy as cnp
import numpy as np

cnp.import_array()

cdef extern from "DracoPy.h" namespace "DracoFunctions":

    cdef enum decoding_status:
        successful, not_draco_encoded, no_position_attribute,
        failed_during_decoding

    cdef enum encoding_status:
        successful_encoding, failed_during_encoding

    cdef struct AttributeData:
        int unique_id
        int num_components
        int data_type
        int attribute_type
        vector[float] float_data
        vector[uint32_t] uint_data
        vector[uint8_t] byte_data

    cdef struct PointCloudObject:
        vector[AttributeData] attributes
        # Encoding options
        bool encoding_options_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status

    cdef struct MeshObject:
        vector[AttributeData] attributes
        # Encoding options
        bool encoding_options_set
        int quantization_bits
        double quantization_range
        vector[double] quantization_origin

        # Represents the decoding success or error message
        decoding_status decode_status
        
        # Mesh-specific
        vector[unsigned int] faces

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
        const vector[float] normals,
        const uint8_t has_normals,
        vector[uint8_t]& unique_ids,
        vector[vector[float]]& attr_float_data,
        vector[vector[uint8_t]]& attr_uint8_data,
        vector[vector[uint16_t]]& attr_uint16_data,
        vector[vector[uint32_t]]& attr_uint32_data,
        vector[int]& attr_data_types,
        vector[int]& attr_num_components
    ) except +

    EncodedObject encode_point_cloud(
        const vector[float] points,
        const int quantization_bits,
        const int compression_level,
        const float quantization_range,
        const float *quantization_origin,
        const bool preserve_order,
        const bool create_metadata,
        const int integer_mark,
        const vector[uint8_t] colors,
        const uint8_t colors_channel
    ) except +
