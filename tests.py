import os
import DracoPy
import pytest

import numpy as np

testdata_directory = "testdata_files"

EXPECTED_POINTS_BUNNY_MESH = 104502 // 3
EXPECTED_POINTS_BUNNY_PTC = 107841 // 3
EXPECTED_FACES_BUNNY = 208353 // 3

def test_decoding_and_encoding_mesh_file():
    with open(os.path.join(testdata_directory, "bunny.drc"), "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert len(mesh_object.points) == EXPECTED_POINTS_BUNNY_MESH
        assert len(mesh_object.faces) == EXPECTED_FACES_BUNNY

        encoding_test = DracoPy.encode_mesh_to_buffer(
            mesh_object.points, mesh_object.faces, 
        )
        with open(os.path.join(testdata_directory, "bunny_test.drc"), "wb") as test_file:
            test_file.write(encoding_test)

    with open(os.path.join(testdata_directory, "bunny_test.drc"), "rb") as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert (mesh_object.encoding_options) is None
        assert len(mesh_object.points) == EXPECTED_POINTS_BUNNY_MESH
        assert len(mesh_object.faces) == EXPECTED_FACES_BUNNY

def test_decoding_and_encoding_mesh_file_integer_positions():
    with open(os.path.join(testdata_directory, "bunny.drc"), "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert len(mesh_object.points) == EXPECTED_POINTS_BUNNY_MESH
        assert len(mesh_object.faces) == EXPECTED_FACES_BUNNY

    points = np.array(mesh_object.points)
    points = points.astype(np.float64)
    points -= np.min(points, axis=0)
    points /= np.max(points, axis=0)
    points *= 2 ** 16
    points = points.astype(np.uint32)

    encoding_test = DracoPy.encode_mesh_to_buffer(
        points, mesh_object.faces, 
        quantization_bits=16,
    )

    encoding_test_float = DracoPy.encode_mesh_to_buffer(
        points.astype(np.float32), mesh_object.faces, 
        quantization_bits=16,
    )
    assert encoding_test != encoding_test_float
    
    encoding_test2 = DracoPy.encode_mesh_to_buffer(
        points.astype(np.int64), mesh_object.faces, 
        quantization_bits=16,
    )
    assert encoding_test == encoding_test2

    mesh_object = DracoPy.decode_buffer_to_mesh(encoding_test)

    assert len(mesh_object.points) == EXPECTED_POINTS_BUNNY_MESH
    assert len(mesh_object.faces) == EXPECTED_FACES_BUNNY

    pts_f = np.array(mesh_object.points)
    pts_f = np.sort(pts_f, axis=0)
    pts_i = np.sort(np.copy(points), axis=0)
    assert np.all(np.isclose(pts_i, pts_f))

def test_decoding_improper_file():
    with open(os.path.join(testdata_directory, "bunny.obj"), "rb") as improper_file:
        file_content = improper_file.read()
        with pytest.raises(DracoPy.FileTypeException):
            DracoPy.decode_buffer_to_mesh(file_content)


def test_metadata():
    with open(os.path.join(testdata_directory, "bunny.drc"), "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        encoding_options = {
            "quantization_bits": 12,
            "compression_level": 3,
            "quantization_range": 1000,
            "quantization_origin": [-100, -100, -100],
            "create_metadata": True,
        }
        encoding_test = DracoPy.encode_mesh_to_buffer(
            mesh_object.points, mesh_object.faces, **encoding_options
        )
        with open(
            os.path.join(testdata_directory, "bunny_test.drc"), "wb"
        ) as test_file:
            test_file.write(encoding_test)

    with open(os.path.join(testdata_directory, "bunny_test.drc"), "rb") as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        eo = mesh_object.encoding_options
        assert (eo) is not None
        assert (eo.quantization_bits) == 12
        assert (eo.quantization_range) == 1000
        assert (eo.quantization_origin) == [-100, -100, -100]


def test_decoding_and_encoding_point_cloud_file():
    with open(
        os.path.join(testdata_directory, "point_cloud_bunny.drc"), "rb"
    ) as draco_file:
        file_content = draco_file.read()
        point_cloud_object = DracoPy.decode_point_cloud_buffer(file_content)
        assert len(point_cloud_object.points) == EXPECTED_POINTS_BUNNY_PTC
        encoding_test = DracoPy.encode_point_cloud_to_buffer(point_cloud_object.points)
        with open(
            os.path.join(testdata_directory, "point_cloud_bunny_test.drc"), "wb"
        ) as test_file:
            test_file.write(encoding_test)

    with open(
        os.path.join(testdata_directory, "point_cloud_bunny_test.drc"), "rb"
    ) as test_file:
        file_content = test_file.read()
        point_cloud_object = DracoPy.decode_point_cloud_buffer(file_content)
        assert (point_cloud_object.encoding_options) is None
        assert len(point_cloud_object.points) == EXPECTED_POINTS_BUNNY_PTC
