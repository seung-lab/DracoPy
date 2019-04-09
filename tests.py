import os
import DracoPy
import pytest

def test_decoding_and_encoding_file():
    expected_points = 104502
    expected_faces = 208353
    with open(os.path.join('./bunny.drc'), 'rb') as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert len(mesh_object.points) == expected_points
        assert len(mesh_object.faces) == expected_faces
        encoding_test = DracoPy.encode_mesh_to_buffer(mesh_object.points, mesh_object.faces)
        with open('bunny_test.drc', 'wb') as test_file:
            test_file.write(encoding_test)

    with open(os.path.join('bunny_test.drc'), 'rb') as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert(mesh_object.encoding_options) is None
        assert len(mesh_object.points) == expected_points
        assert len(mesh_object.faces) == expected_faces

def test_decoding_improper_file():
    with open(os.path.join('./bunny.obj'), 'rb') as improper_file:
        file_content = improper_file.read()
        with pytest.raises(DracoPy.FileTypeException):
            DracoPy.decode_buffer_to_mesh(file_content)

def test_metadata():
    with open(os.path.join('./bunny.drc'), 'rb') as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        encoding_options = {
            'quantization_bits': 12,
            'compression_level': 3,
            'quantization_range': 1000,
            'quantization_origin': [-100, -100, -100],
            'create_metadata': True
        }
        encoding_test = DracoPy.encode_mesh_to_buffer(mesh_object.points, mesh_object.faces, **encoding_options)
        with open('bunny_test.drc', 'wb') as test_file:
            test_file.write(encoding_test)
    
    with open(os.path.join('bunny_test.drc'), 'rb') as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        eo = mesh_object.encoding_options
        assert(eo) is not None
        assert(eo.quantization_bits) == 12
        assert(eo.quantization_range) == 1000
        assert(eo.quantization_origin) == [-100, -100, -100]