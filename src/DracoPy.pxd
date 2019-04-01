from libcpp.vector cimport vector
from libc.stdint cimport uint32_t
from libcpp cimport bool

cdef extern from "DracoPy.h" namespace "DracoFunctions":

    cdef enum decoding_status:
        successful, not_draco_encoded, no_position_attribute, 
        failed_during_decoding
    
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


    MeshObject decode_buffer(const char *buffer, size_t buffer_len) except +

    vector[unsigned char] encode_mesh(vector[float] points, vector[uint32_t] faces, int quantization_bits,
        int compression_level, float quantization_range, const float *quantization_origin) except +
