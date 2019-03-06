from libcpp.vector cimport vector
from libc.stdint cimport uint32_t

cdef extern from "DracoPy.h" namespace "DracoFunctions":
    
    cdef struct MeshObject:
        vector[float] points
        vector[unsigned int] faces

        # TODO: add support for normals, which are not currently supported.
        vector[float] normals

    MeshObject decode_buffer(const char *buffer, size_t buffer_len) except +

    vector[unsigned char] encode_mesh(vector[float] points, vector[uint32_t] faces, int quantization_bits,
        int compression_level, float quantization_range, const float *quantization_origin) except +
