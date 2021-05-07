import os
import DracoPy

testdata_directory = "testdata_files"

with open(os.path.join(testdata_directory, "Avocado.bin"), "rb") as draco_file:
  file_content = draco_file.read()
  mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
  print('number of faces in original file: {0}'.format(len(mesh_object.tex_coord)))
  print('number of normal in original file: {0}'.format(len(mesh_object.normals)))
  encoding_test = DracoPy.encode_mesh_to_buffer(mesh_object.points, mesh_object.face)
  with open('bunny_test.drc', 'wb') as test_file:
    test_file.write(encoding_test)

with open('bunny_test.drc', 'rb') as test_file:
  file_content = test_file.read()
  mesh_object = DracoPy.decode_buffer_to_mesh(file_content)
  print('number of points in test file: {0}'.format(len(mesh_object.points)))
  print('number of faces in test file: {0}'.format(len(mesh_object.faces)))
