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
        assert len(mesh_object.points) == expected_points
        assert len(mesh_object.faces) == expected_faces

def test_decoding_improper_file():
    with open(os.path.join('./bunny.obj'), 'rb') as improper_file:
        file_content = improper_file.read()
        with pytest.raises(DracoPy.FileTypeException):
            DracoPy.decode_buffer_to_mesh(file_content)