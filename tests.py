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

    # test preserve_order
    np.random.shuffle(mesh.faces)
    encoding_test3 = DracoPy.encode(mesh.points, mesh.faces, compression_level=10,
                                    preserve_order=True)
    mesh_decode = DracoPy.decode(encoding_test3)
    assert np.allclose(mesh.points, mesh_decode.points)
    assert np.allclose(mesh.faces, mesh_decode.faces)
    # color is None
    assert mesh_decode.colors is None, "colors should not present"

    colors = np.random.randint(0, 255, [mesh.points.shape[0], 16]).astype(np.uint8)

    # test extreme quantization
    encoding_test4 = DracoPy.encode(mesh.points, mesh.faces, compression_level=10,
                                    quantization_bits=1, colors=colors,
                                    preserve_order=True)
    mesh_decode = DracoPy.decode(encoding_test4)
    assert np.array_equal(colors, mesh_decode.colors), "colors decode result is wrong"

    # Setting quantization_bits 26 here. Larger value causes MemoryError on 32bit systems.
    encoding_test5 = DracoPy.encode(mesh.points, mesh.faces, compression_level=1,
                                    quantization_bits=26, colors=colors)
    mesh_decode = DracoPy.decode(encoding_test5)
    assert mesh_decode.colors is not None, "colors should present"

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

    encoding_test_uint = DracoPy.encode(
        points, mesh_object.faces,
        quantization_bits=16,
    )

    encoding_test_int = DracoPy.encode(
        points.astype(np.int64), mesh_object.faces,
        quantization_bits=16,
    )

    encoding_test_float = DracoPy.encode(
        points.astype(np.float32), mesh_object.faces,
        quantization_bits=16,
    )
    assert encoding_test_uint != encoding_test_float
    assert encoding_test_uint != encoding_test_int
    assert encoding_test_int != encoding_test_float

    encoding_test_uint64 = DracoPy.encode(
        points.astype(np.uint64), mesh_object.faces,
        quantization_bits=16,
    )
    assert encoding_test_uint == encoding_test_uint64

    mesh_object = DracoPy.decode(encoding_test_uint)

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

        # test preserve_order
        np.random.shuffle(point_cloud_object.points)
        colors = np.random.randint(0, 255, [point_cloud_object.points.shape[0], 127]).astype(np.uint8)
        encoding_test2 = DracoPy.encode(point_cloud_object.points, compression_level=10,
                                        quantization_bits=26, preserve_order=True, colors=colors)
        ptc_decode = DracoPy.decode(encoding_test2)
        assert np.allclose(point_cloud_object.points, ptc_decode.points)
        assert np.array_equal(colors, ptc_decode.colors)

        # test extreme quantization
        encoding_test3 = DracoPy.encode(point_cloud_object.points, compression_level=10,
                                        quantization_bits=1)
        ptc_decode = DracoPy.decode(encoding_test3)
        assert ptc_decode.colors is None, "colors should not present"

    with open(
        os.path.join(testdata_directory, "point_cloud_bunny_test.drc"), "rb"
    ) as test_file:
        file_content = test_file.read()
        point_cloud_object = DracoPy.decode(file_content)
        assert (point_cloud_object.encoding_options) is None
        assert len(point_cloud_object.points) == EXPECTED_POINTS_BUNNY_PTC
