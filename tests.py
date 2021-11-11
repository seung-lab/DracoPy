import os
from typing import Tuple, List, Dict

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
          |
         (0)--------(1)
         /
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

def create_points_map(in_points: List[int],
                      out_points: List[int]) -> Dict[int, int]:
    index_map = {
        tuple(out_points[i * 3:(i + 1) * 3]): i
        for i in range(len(out_points) // 3)
    }
    for i in range(len(in_points) // 3):
        in_vertex = tuple(in_points[i * 3:(i + 1) * 3])
        out_index = index_map.pop(in_vertex)
        in_index = i
        index_map[in_index] = out_index
    for key in index_map:
        assert isinstance(key, int), f"input point {key} is not in output"
    return index_map


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
            mesh_object.points, mesh_object.faces)
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
    buffer = DracoPy.encode_mesh_to_buffer(points, faces)
    mesh = DracoPy.decode_buffer_to_mesh(buffer)
    print(mesh)


def test_encode_decode_geometry_attributes():
    # prepare input data
    points, faces = create_tetrahedron()
    metadatas = [
        {
            "entries": {},
            "sub_metadata_ids": {},
        }
        for _ in range(2)
    ]
    attribute = {
        "data": {
            i: int.to_bytes(4 - i, 4, "little", signed=False)
            for i in range(4)
        },
        "datatype": 6,
        "dimension": 1,
        "metadata_id": 1,
    }
    geometry_metadata = {
        "metadata_id": 0,
        "generic_attributes": [attribute],
    }
    # encode - decode
    buffer = DracoPy.encode_mesh_to_buffer(
        points, faces, metadatas=metadatas, geometry_metadata=geometry_metadata)
    mesh = DracoPy.decode_buffer_to_mesh(buffer, True)
    points_map = create_points_map(points, mesh.points)
    # validate results
    out_generic_attributes = mesh.geometry_metadata["generic_attributes"]
    assert len(out_generic_attributes) == 1
    out_attribute = out_generic_attributes[0]
    assert out_attribute["datatype"] == attribute["datatype"]
    assert out_attribute["dimension"] == attribute["dimension"]
    assert out_attribute["metadata_id"] == attribute["metadata_id"]
    assert len(out_attribute["data"]) == len(attribute["data"])
    for in_index, point_value in attribute["data"].items():
        out_index = points_map[in_index]
        assert out_attribute["data"][out_index] == point_value


def test_encode_decode_metadata_entries():
    # prepare input data
    points, faces = create_tetrahedron()
    geometry_metadata = {
        "entries": {b"name": b"global_geometry_metadata_name"},
        "sub_metadata_ids": {b"custom_metadata_name": 1},
    }
    custom_submetadata = {
        "entries": {b"name": b"custom_attribute_name"},
        "sub_metadata_ids": {},
    }
    metadatas = [
        geometry_metadata,
        custom_submetadata,
    ]
    geometry_metadata_specific = {
        "metadata_id": 0,
        "generic_attributes": [],
    }
    # encode - decode
    buffer = DracoPy.encode_mesh_to_buffer(
        points, faces, metadatas=metadatas,
        geometry_metadata=geometry_metadata_specific)
    mesh = DracoPy.decode_buffer_to_mesh(buffer)
    # validate results
    out_metadatas = mesh.metadatas
    assert len(out_metadatas) == len(metadatas)
    out_geometry_metadata = out_metadatas[mesh.geometry_metadata["metadata_id"]]
    assert out_geometry_metadata["entries"] == geometry_metadata["entries"]
    assert list(out_geometry_metadata["sub_metadata_ids"]) == \
           list(geometry_metadata["sub_metadata_ids"])
    custom_metadata_id = out_geometry_metadata["sub_metadata_ids"][b"custom_metadata_name"]
    custom_metadata = out_metadatas[custom_metadata_id]
    assert custom_metadata == metadatas[1]

# test_encode_decode_metadata_entries()