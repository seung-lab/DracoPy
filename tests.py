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
        mesh = DracoPy.decode(draco_file.read())
    
    assert len(mesh.points) == EXPECTED_POINTS_BUNNY_MESH
    assert len(mesh.faces) == EXPECTED_FACES_BUNNY
    assert len(mesh.normals) == 0

    with open(os.path.join(testdata_directory, "bunny_normals.drc"), "rb") as draco_file:
        mesh = DracoPy.decode(draco_file.read())
    
    assert len(mesh.points) == EXPECTED_POINTS_BUNNY_MESH
    assert len(mesh.faces) == EXPECTED_FACES_BUNNY
    assert len(mesh.normals) == EXPECTED_POINTS_BUNNY_MESH

    encoding_test1 = DracoPy.encode(mesh.points, mesh.faces)
    encoding_test2 = DracoPy.encode(mesh.points.flatten(), mesh.faces)

    assert encoding_test1 == encoding_test2
    encoding_test = encoding_test1

    with open(os.path.join(testdata_directory, "bunny_test.drc"), "wb") as test_file:
        test_file.write(encoding_test)

    with open(os.path.join(testdata_directory, "bunny_test.drc"), "rb") as test_file:
        mesh = DracoPy.decode(test_file.read())
        assert (mesh.encoding_options) is None
        assert len(mesh.points) == EXPECTED_POINTS_BUNNY_MESH
        assert len(mesh.faces) == EXPECTED_FACES_BUNNY

def test_decoding_and_encoding_mesh_file_integer_positions():
    with open(os.path.join(testdata_directory, "bunny.drc"), "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode(file_content)
        assert len(mesh_object.points) == EXPECTED_POINTS_BUNNY_MESH
        assert len(mesh_object.faces) == EXPECTED_FACES_BUNNY

    points = np.array(mesh_object.points)
    points = points.astype(np.float64)
    points -= np.min(points, axis=0)
    points /= np.max(points, axis=0)
    points *= 2 ** 16
    points = points.astype(np.uint32)

    encoding_test = DracoPy.encode(
        points, mesh_object.faces, 
        quantization_bits=16,
    )

    encoding_test_float = DracoPy.encode(
        points.astype(np.float32), mesh_object.faces, 
        quantization_bits=16,
    )
    assert encoding_test != encoding_test_float
    
    encoding_test2 = DracoPy.encode(
        points.astype(np.int64), mesh_object.faces, 
        quantization_bits=16,
    )
    assert encoding_test == encoding_test2

    mesh_object = DracoPy.decode(encoding_test)

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
            DracoPy.decode(file_content)


def test_metadata():
    with open(os.path.join(testdata_directory, "bunny.drc"), "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode(file_content)
        encoding_options = {
            "quantization_bits": 12,
            "compression_level": 3,
            "quantization_range": 1000,
            "quantization_origin": [-100, -100, -100],
            "create_metadata": True,
        }
        encoding_test = DracoPy.encode(
            mesh_object.points, mesh_object.faces, **encoding_options
        )
        with open(
            os.path.join(testdata_directory, "bunny_test.drc"), "wb"
        ) as test_file:
            test_file.write(encoding_test)

    with open(os.path.join(testdata_directory, "bunny_test.drc"), "rb") as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode(file_content)
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
        point_cloud_object = DracoPy.decode(file_content)
        assert len(point_cloud_object.points) == EXPECTED_POINTS_BUNNY_PTC
        encoding_test = DracoPy.encode(point_cloud_object.points)
        with open(
            os.path.join(testdata_directory, "point_cloud_bunny_test.drc"), "wb"
        ) as test_file:
            test_file.write(encoding_test)

    with open(
        os.path.join(testdata_directory, "point_cloud_bunny_test.drc"), "rb"
    ) as test_file:
        file_content = test_file.read()
        point_cloud_object = DracoPy.decode(file_content)
        assert (point_cloud_object.encoding_options) is None
        assert len(point_cloud_object.points) == EXPECTED_POINTS_BUNNY_PTC
