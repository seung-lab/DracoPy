import os
from typing import Tuple, List

import DracoPy
import pytest

testdata_directory = "testdata_files"

###############################################
############## Utils For Tests ################
###############################################

def create_tetrahedron() -> Tuple[List[float], List[int]]:
    """
       Tetrader
         (3)
          |
          |
          |
         (0)--------(1)
         /
        /
      (2)
    """
    points = [
    #   X,  Y,  Z
        0., 0., 0.,  # 0
        1., 0., 0.,  # 1
        0., 1., 0.,  # 2
        1., 1., 1.,  # 3
    ]
    faces = [
        0, 1, 2,
        0, 1, 3,
        1, 2, 3,
        2, 0, 3,
    ]
    return points, faces


def create_empty_geometry_metadata() -> dict:
    return {
        "metadata_id": 0,
        "attribute_metadatas": [],
    }


###############################################
################ Tests itselves ###############
###############################################


def test_decoding_and_encoding_mesh_file():
    expected_points = 104502
    expected_faces = 208353
    with open(os.path.join(testdata_directory, "bunny.drc"), "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert len(mesh_object.points) == expected_points
        assert len(mesh_object.faces) == expected_faces
        encoding_test = DracoPy.encode_mesh_to_buffer(
            mesh_object.points, mesh_object.faces, mesh_object.data_struct,
        )
        with open(os.path.join(testdata_directory, "bunny_test.drc"),
                  "wb") as test_file:
            test_file.write(encoding_test)

    with open(os.path.join(testdata_directory, "bunny_test.drc"),
              "rb") as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        assert (mesh_object.encoding_options) is None
        assert len(mesh_object.points) == expected_points
        assert len(mesh_object.faces) == expected_faces


def test_decoding_improper_file():
    with open(os.path.join(testdata_directory, "bunny.obj"),
              "rb") as improper_file:
        file_content = improper_file.read()
        with pytest.raises(DracoPy.FileTypeException):
            DracoPy.decode_buffer_to_mesh(file_content)


def test_metadata():
    with open(os.path.join(testdata_directory, "bunny.drc"),
              "rb") as draco_file:
        file_content = draco_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        encoding_options = {
            "mesh_object": mesh_object.data_struct,
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

    with open(os.path.join(testdata_directory, "bunny_test.drc"),
              "rb") as test_file:
        file_content = test_file.read()
        mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
        eo = mesh_object.encoding_options
        assert (eo) is not None
        assert (eo.quantization_bits) == 12
        assert (eo.quantization_range) == 1000
        assert (eo.quantization_origin) == [-100, -100, -100]


def test_decoding_and_encoding_point_cloud_file():
    expected_points = 107841
    with open(
            os.path.join(testdata_directory, "point_cloud_bunny.drc"), "rb"
    ) as draco_file:
        file_content = draco_file.read()
        point_cloud_object = DracoPy.decode_point_cloud_buffer(file_content)
        assert len(point_cloud_object.points) == expected_points
        encoding_test = DracoPy.encode_point_cloud_to_buffer(
            point_cloud_object.points)
        with open(
                os.path.join(testdata_directory, "point_cloud_bunny_test.drc"),
                "wb"
        ) as test_file:
            test_file.write(encoding_test)

    with open(
            os.path.join(testdata_directory, "point_cloud_bunny_test.drc"), "rb"
    ) as test_file:
        file_content = test_file.read()
        point_cloud_object = DracoPy.decode_point_cloud_buffer(file_content)
        assert (point_cloud_object.encoding_options) is None
        assert len(point_cloud_object.points) == expected_points


def test_encode_decode_tetrahedron():
    points, faces = create_tetrahedron()
    buffer = DracoPy.encode_mesh_to_buffer(points, faces, [],
                                           create_empty_geometry_metadata(),
                                           [])
    mesh = DracoPy.decode_buffer_to_mesh(buffer)
    print(mesh)


def test_encode_decode_tetrahedron_attributes():
    points, faces = create_tetrahedron()
    attributes = [

    ]
    metadatas = [
        {
            "entries": {
                b"name": b"global_geometry_metadata_name",
            },
            "sub_metadata_ids": {
            },
        },
        {
            "entries": {
                b"name": b"custom_attribute_name"
            },
            "sub_metadata_ids": {},
        },
    ]
    geometry_metadata = {
        "metadata_id": 0,
        "generic_attributes": [
            {
                "data": {
                    i: int.to_bytes(i, 4, "little", signed=False)
                                    for i in range(4)
                },
                "datatype": 6,
                "dimension": 1,
                "metadata_id": 1,
            }
        ],
    }
    buffer = DracoPy.encode_mesh_to_buffer(points, faces, metadatas,
                                           geometry_metadata)
    mesh = DracoPy.decode_buffer_to_mesh(buffer)
    print(mesh)


# test_encode_decode_tetrahedron()
test_encode_decode_tetrahedron_attributes()