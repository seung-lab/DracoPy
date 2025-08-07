#ifndef __DRACOPY_H__
#define __DRACOPY_H__

#include<algorithm>
#include<cmath>
#include<vector>
#include<cstddef>
#include "draco/compression/decode.h"
#include "draco/compression/encode.h"
#include "draco/compression/config/compression_shared.h"
#include "draco/core/status_or.h"
#include "draco/core/encoder_buffer.h"
#include "draco/core/vector_d.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/point_cloud/point_cloud_builder.h"

namespace DracoFunctions {

  enum decoding_status {
    successful,
    not_draco_encoded,
    no_position_attribute,
    failed_during_decoding
  };
  enum encoding_status {
    successful_encoding,
    failed_during_encoding
  };

  struct AttributeData {
    int unique_id;
    int num_components;
    int data_type;  // draco::DataType as int
    int attribute_type;  // draco::GeometryAttribute::Type as int
    std::vector<float> float_data;    // For float data
    std::vector<uint32_t> uint_data;  // For integer data
    std::vector<uint8_t> byte_data;   // For byte data
  };

  struct PointCloudObject {
    std::vector<AttributeData> attributes; 

    // Encoding options stored in metadata
    bool encoding_options_set;
    bool colors_set;
    int quantization_bits;
    double quantization_range;
    std::vector<double> quantization_origin;

    decoding_status decode_status;
  };

  struct MeshObject : PointCloudObject {
    std::vector<unsigned int> faces;
  };

  struct EncodedObject {
    std::vector<unsigned char> buffer;
    encoding_status encode_status;
  };


#define CHECK_STATUS(statusor, obj) \
    if (!(statusor).ok()) {\
      std::string status_string = (statusor).status().error_msg_string(); \
      if (\
        status_string.compare("Not a Draco file.") \
        || status_string.compare("Failed to parse Draco header.")) {\
\
        (obj).decode_status = not_draco_encoded;\
      }\
      else {\
        (obj).decode_status = failed_during_decoding;\
      }\
      return (obj);\
    }

  MeshObject decode_buffer(const char *buffer, std::size_t buffer_len) {
    MeshObject meshObject;
    draco::DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);

    auto type_statusor = draco::Decoder::GetEncodedGeometryType(&decoderBuffer);
    CHECK_STATUS(type_statusor, meshObject)
    draco::EncodedGeometryType geotype = std::move(type_statusor).value();

    if (geotype == draco::EncodedGeometryType::INVALID_GEOMETRY_TYPE) {
      meshObject.decode_status = not_draco_encoded;
      return meshObject;
    }

    draco::Decoder decoder;
    std::unique_ptr<draco::Mesh> in_mesh;
    std::unique_ptr<draco::PointCloud> in_pointcloud;
    draco::Mesh *mesh;

    if (geotype == draco::EncodedGeometryType::POINT_CLOUD) {
      auto statusor = decoder.DecodePointCloudFromBuffer(&decoderBuffer);
      CHECK_STATUS(statusor, meshObject)
      in_pointcloud = std::move(statusor).value();
      // This is okay because draco::Mesh is a subclass of
      // draco::PointCloud
      mesh = static_cast<draco::Mesh*>(in_pointcloud.get());
    }
    else if (geotype == draco::EncodedGeometryType::TRIANGULAR_MESH) {
      auto statusor = decoder.DecodeMeshFromBuffer(&decoderBuffer);
      CHECK_STATUS(statusor, meshObject)
      in_mesh = std::move(statusor).value();
      mesh = in_mesh.get();
    }
    else {
      throw std::runtime_error("Should never be reached.");
    }


    // Get faces if it's a mesh
    if (geotype == draco::EncodedGeometryType::TRIANGULAR_MESH) {
      meshObject.faces.reserve(3 * mesh->num_faces());
      for (draco::FaceIndex f(0); f < mesh->num_faces(); ++f) {
        const auto& face = mesh->face(f);
        meshObject.faces.push_back(face[0].value());
        meshObject.faces.push_back(face[1].value());
        meshObject.faces.push_back(face[2].value());
      }
    }

    // Collect all attributes in a unified way
    // std::cout << "DEBUG: Collecting all attributes, total: " << mesh->num_attributes() << std::endl;
    for (int att_id = 0; att_id < mesh->num_attributes(); ++att_id) {
      const auto *const att = mesh->attribute(att_id);
      
      AttributeData attr;
      attr.unique_id = att->unique_id();
      attr.num_components = att->num_components();
      attr.data_type = static_cast<int>(att->data_type());
      attr.attribute_type = static_cast<int>(att->attribute_type());

      // Extract data based on data type
      const int num_values = mesh->num_points() * att->num_components();

      switch (att->data_type()) {
        case draco::DT_FLOAT32: {
          attr.float_data.reserve(num_values);
          std::unique_ptr<float[]> values(new float[att->num_components()]);
          for (draco::PointIndex v(0); v < mesh->num_points(); ++v) {
            if (att->ConvertValue<float>(att->mapped_index(v), att->num_components(), values.get())) {
              for (int c = 0; c < att->num_components(); ++c) {
                attr.float_data.push_back(values[c]);
              }
            }
          }
          break;
        }
        case draco::DT_UINT8: {
          attr.byte_data.reserve(num_values);
          std::unique_ptr<uint8_t[]> values(new uint8_t[att->num_components()]);
          for (draco::PointIndex v(0); v < mesh->num_points(); ++v) {
            if (att->ConvertValue<uint8_t>(att->mapped_index(v), att->num_components(), values.get())) {
              for (int c = 0; c < att->num_components(); ++c) {
                attr.byte_data.push_back(values[c]);
              }
            }
          }
          break;
        }
        case draco::DT_UINT16: {
          attr.uint_data.reserve(num_values);
          std::unique_ptr<uint16_t[]> values(new uint16_t[att->num_components()]);
          for (draco::PointIndex v(0); v < mesh->num_points(); ++v) {
            if (att->ConvertValue<uint16_t>(att->mapped_index(v), att->num_components(), values.get())) {
              for (int c = 0; c < att->num_components(); ++c) {
                attr.uint_data.push_back(static_cast<uint32_t>(values[c]));
              }
            }
          }
          break;
        }
        case draco::DT_UINT32: {
          attr.uint_data.reserve(num_values);
          std::unique_ptr<uint32_t[]> values(new uint32_t[att->num_components()]);
          for (draco::PointIndex v(0); v < mesh->num_points(); ++v) {
            if (att->ConvertValue<uint32_t>(att->mapped_index(v), att->num_components(), values.get())) {
              for (int c = 0; c < att->num_components(); ++c) {
                attr.uint_data.push_back(values[c]);
              }
            }
          }
          break;
        }
        default: {
          // For other data types, try to convert to float as fallback
          attr.float_data.reserve(num_values);
          std::unique_ptr<float[]> values(new float[att->num_components()]);
          for (draco::PointIndex v(0); v < mesh->num_points(); ++v) {
            if (att->ConvertValue<float>(att->mapped_index(v), att->num_components(), values.get())) {
              for (int c = 0; c < att->num_components(); ++c) {
                attr.float_data.push_back(values[c]);
              }
            }
          }
          break;
        }
      }
      
      meshObject.attributes.push_back(attr);
      // std::cout << "DEBUG: Added attribute " << att_id << " with type " << static_cast<int>(att->attribute_type()) << std::endl;
    }

    // Set encoding options from metadata
    const draco::GeometryMetadata *metadata = mesh->GetMetadata();
    meshObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(meshObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(meshObject.quantization_range)) &&
          metadata->GetEntryDoubleArray("quantization_origin", &(meshObject.quantization_origin))) {
          meshObject.encoding_options_set = true;
      }
    }

    meshObject.decode_status = successful;
    return meshObject;
  }

  void setup_encoder_and_metadata(draco::PointCloud *point_cloud_or_mesh, draco::Encoder &encoder, int compression_level, int quantization_bits, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int speed = 10 - compression_level;
    encoder.SetSpeedOptions(speed, speed);
    std::unique_ptr<draco::GeometryMetadata> metadata = std::unique_ptr<draco::GeometryMetadata>(new draco::GeometryMetadata());
    if (quantization_origin == NULL || quantization_range <= 0.f) {
      // @zeruniverse All quantization_range <= 0.f is useless, see
      //    https://github.com/google/draco/blob/master/src/draco/attributes/attribute_quantization_transform.cc#L160-L170
      encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, quantization_bits);
    }
    else {
      encoder.SetAttributeExplicitQuantization(draco::GeometryAttribute::POSITION, quantization_bits, 3, quantization_origin, quantization_range);
      if (create_metadata) {
        metadata->AddEntryDouble("quantization_range", quantization_range);
        std::vector<double> quantization_origin_vec;
        for (int i = 0; i < 3; i++) {
          quantization_origin_vec.push_back(quantization_origin[i]);
        }
        metadata->AddEntryDoubleArray("quantization_origin", quantization_origin_vec);
      }
    }
    if (create_metadata) {
      metadata->AddEntryInt("quantization_bits", quantization_bits);
      point_cloud_or_mesh->AddMetadata(std::move(metadata));
    }
  }

  EncodedObject encode_mesh(
    const std::vector<float> &points,
    const std::vector<unsigned int> &faces,
    const int quantization_bits,
    const int compression_level,
    const float quantization_range,
    const float *quantization_origin,
    const bool preserve_order,
    const bool create_metadata,
    const int integer_mark,
    const std::vector<uint8_t> &colors,
    const uint8_t colors_channel,
    const std::vector<float> &tex_coord,
    const uint8_t tex_coord_channel,
    const std::vector<float> &normals,
    const uint8_t has_normals,
    std::vector<uint8_t>& unique_ids,
    std::vector<std::vector<float>>& attr_float_data,
    std::vector<std::vector<uint8_t>>& attr_uint8_data,
    std::vector<std::vector<uint16_t>>& attr_uint16_data,
    std::vector<std::vector<uint32_t>>& attr_uint32_data,
    std::vector<int>& attr_data_types,
    std::vector<int>& attr_num_components
  ) {
    // @zeruniverse TriangleSoupMeshBuilder will cause problems when
    //    preserve_order=True due to vertices merging.
    //    In order to support preserve_order, we need to build mesh
    //    manually.
    draco::Mesh mesh; //Initialize a draco mesh

    // Process vertices
    const size_t num_pts = points.size() / 3;
    mesh.set_num_points(num_pts);
    draco::GeometryAttribute positions_attr;
    if (integer_mark == 1) {
      positions_attr.Init(draco::GeometryAttribute::POSITION, // Attribute type
                          nullptr,                            // data buffer
                          3,                                  // number of components
                          draco::DT_INT32,                    // data type
                          false,                              // normalized
                          sizeof(int32_t) * 3,                // byte stride
                          0);                                 // byte offset
    } else if (integer_mark == 2) {
      positions_attr.Init(draco::GeometryAttribute::POSITION, // Attribute type
                          nullptr,                            // data buffer
                          3,                                  // number of components
                          draco::DT_UINT32,                   // data type
                          false,                              // normalized
                          sizeof(uint32_t) * 3,               // byte stride
                          0);                                 // byte offset
    } else {
      positions_attr.Init(draco::GeometryAttribute::POSITION, // Attribute type
                          nullptr,                            // data buffer
                          3,                                  // number of components
                          draco::DT_FLOAT32,                  // data type
                          false,                              // normalized
                          sizeof(float) * 3,                  // byte stride
                          0);                                 // byte offset
    }
    int color_att_id = -1;
    if(colors_channel) {
      draco::GeometryAttribute colors_attr;
      colors_attr.Init(draco::GeometryAttribute::COLOR,    // Attribute type
                       nullptr,                            // data buffer
                       colors_channel,                     // number of components
                       draco::DT_UINT8,                    // data type
                       true,                               // normalized
                       sizeof(uint8_t) * colors_channel,   // byte stride
                       0);                                 // byte offset
      color_att_id = mesh.AddAttribute(colors_attr, true, num_pts);
    }
    int tex_coord_att_id = -1;
    if(tex_coord_channel) {
      draco::GeometryAttribute tex_coord_attr;
      tex_coord_attr.Init(draco::GeometryAttribute::TEX_COORD, // Attribute type
                          nullptr,                             // data buffer
                          tex_coord_channel,                   // number of components
                          draco::DT_FLOAT32,                   // data type
                          true,                                // normalized
                          sizeof(float) * tex_coord_channel,   // byte stride
                          0);                                  // byte offset
      tex_coord_att_id = mesh.AddAttribute(tex_coord_attr, true, num_pts);
    }

    int normal_att_id = -1;
    if(has_normals) {
      draco::GeometryAttribute normal_attr;
      normal_attr.Init(draco::GeometryAttribute::NORMAL,    // Attribute type
                       nullptr,                             // data buffer
                       3,                                   // number of components (normals are 3D vectors)
                       draco::DT_FLOAT32,                   // data type
                       false,                               // normalized
                       sizeof(float) * 3,                   // byte stride
                       0);                                  // byte offset
      normal_att_id = mesh.AddAttribute(normal_attr, true, num_pts);
    }



    // GENERIC ATTRIBUTES
    std::vector<int> generic_attr_ids;
    generic_attr_ids.reserve(unique_ids.size());

    //std::cout << "DEBUG: attr_names size: " << unique_ids.size() << std::endl;

    for (size_t i = 0; i < unique_ids.size(); ++i) {
      draco::GeometryAttribute generic_attr;
      draco::DataType dtype = static_cast<draco::DataType>(attr_data_types[i]);
      int num_components = attr_num_components[i];
      if (dtype != draco::DT_FLOAT32 && dtype != draco::DT_UINT8 && dtype != draco::DT_UINT16 && dtype != draco::DT_UINT32) {
        // Unsupported data type, skip
        // std::cout << "DEBUG: Unsupported attribute data type for " << unique_ids[i] << std::endl;
        generic_attr_ids.push_back(-1);
        continue;
      }
      generic_attr.Init(draco::GeometryAttribute::GENERIC, nullptr, num_components, dtype, false, 0, 0);
      int generic_att_id = mesh.AddAttribute(generic_attr, true, num_pts);
      mesh.attribute(generic_att_id)->set_unique_id(unique_ids[i]);
      if (generic_att_id == -1) {
        // Failed to add attribute, skip
        // std::cout << "DEBUG: Failed to add attribute " << unique_ids[i] << std::endl;
        generic_attr_ids.push_back(-1);
        continue;
      }
      generic_attr_ids.push_back(generic_att_id);
    }


    // std::cout << "DEBUG: Encode all generic attributes, total: " << attr_ids.size() << std::endl;

    const int pos_att_id = mesh.AddAttribute(positions_attr, true, num_pts);
    std::vector<int32_t> pts_int32;
    std::vector<uint32_t> pts_uint32;
    if (integer_mark == 1) {
      pts_int32.reserve(points.size());
      std::transform(points.begin(), points.end(), std::back_inserter(pts_int32), [](float x) {
        return lrint(x);
      });
    } else if (integer_mark == 2) {
      pts_uint32.reserve(points.size());
      std::transform(points.begin(), points.end(), std::back_inserter(pts_uint32), [](float x) {
        return (x <= 0.f)? 0: (uint32_t)(x + 0.5);
      });
    }


    for (size_t i = 0; i < num_pts; ++i) {
      if (integer_mark == 1) {
        mesh.attribute(pos_att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &pts_int32[i * 3ul]);
      } else if (integer_mark == 2) {
        mesh.attribute(pos_att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &pts_uint32[i * 3ul]);
      } else {
        mesh.attribute(pos_att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &points[i * 3ul]);
      }

      if(colors_channel){
        mesh.attribute(color_att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &colors[i * colors_channel]);
      }
      if(tex_coord_channel){
        mesh.attribute(tex_coord_att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &tex_coord[i * tex_coord_channel]);
      }
      if(has_normals){
        mesh.attribute(normal_att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &normals[i * 3]);
      }


      // GENERIC ATTRIBUTES
      for (size_t j = 0; j < generic_attr_ids.size(); ++j) {
        const auto &unique_id = unique_ids[j];
        const auto &float_data = attr_float_data[j];
        const auto &uint8_data = attr_uint8_data[j];
        const auto &uint16_data = attr_uint16_data[j];
        const auto &uint32_data = attr_uint32_data[j];

        if (generic_attr_ids[j] == -1) {
          // Skip if attribute was not added
          continue;
        } 

        if (attr_data_types[j] == draco::DT_FLOAT32) {
          mesh.attribute(generic_attr_ids[j])->SetAttributeValue(draco::AttributeValueIndex(i), &float_data[i * attr_num_components[j]]);
        } else if (attr_data_types[j] == draco::DT_UINT8) {
          mesh.attribute(generic_attr_ids[j])->SetAttributeValue(draco::AttributeValueIndex(i), &uint8_data[i * attr_num_components[j]]);
        } else if (attr_data_types[j] == draco::DT_UINT16) {
          mesh.attribute(generic_attr_ids[j])->SetAttributeValue(draco::AttributeValueIndex(i), &uint16_data[i * attr_num_components[j]]);
        } else if (attr_data_types[j] == draco::DT_UINT32) {
          mesh.attribute(generic_attr_ids[j])->SetAttributeValue(draco::AttributeValueIndex(i), &uint32_data[i * attr_num_components[j]]);
        }
      }
    }

    const size_t num_faces = faces.size() / 3;
    for (size_t i = 0; i < num_faces; ++i) {
      mesh.AddFace(
          draco::Mesh::Face{draco::PointIndex(faces[3 * i]),
                            draco::PointIndex(faces[3 * i + 1]),
                            draco::PointIndex(faces[3 * i + 2])});
    }

    // deduplicate
    if (!preserve_order && mesh.DeduplicateAttributeValues()) {
      mesh.DeduplicatePointIds();
    }

    draco::Encoder encoder;
    setup_encoder_and_metadata(
      &mesh, encoder, compression_level,
      quantization_bits, quantization_range,
      quantization_origin, create_metadata
    );
    if (preserve_order) {
      encoder.SetEncodingMethod(draco::MESH_SEQUENTIAL_ENCODING);
    }

    draco::EncoderBuffer buffer;
    const draco::Status status = encoder.EncodeMeshToBuffer(mesh, &buffer);
    EncodedObject encodedMeshObject;
    encodedMeshObject.buffer = *((std::vector<unsigned char> *)buffer.buffer());

    if (status.ok()) {
      encodedMeshObject.encode_status = successful_encoding;
    }
    else {
      std::cerr << "Draco encoding error: " << status.error_msg_string() << std::endl;
      encodedMeshObject.encode_status = failed_during_encoding;
    }

    return encodedMeshObject;
  }

  EncodedObject encode_point_cloud(
    const std::vector<float> &points, const int quantization_bits,
    const int compression_level, const float quantization_range,
    const float *quantization_origin, const bool preserve_order,
    const bool create_metadata, const int integer_mark,
    const std::vector<uint8_t> &colors,
    const uint8_t colors_channel
  ) {
    int num_points = points.size() / 3;
    draco::PointCloudBuilder pcb;
    pcb.Start(num_points);

    auto dtype = (integer_mark == 1)
      ? draco::DataType::DT_INT32
      : (
        (integer_mark == 2)
          ? draco::DataType::DT_UINT32
          : draco::DataType::DT_FLOAT32
      );

    const int pos_att_id = pcb.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, dtype
    );

    if(colors_channel){
      const int color_att_id = pcb.AddAttribute(
        draco::GeometryAttribute::COLOR, colors_channel, draco::DataType::DT_UINT8
      );
      for (draco::PointIndex i(0); i < num_points; i++) {
        pcb.SetAttributeValueForPoint(pos_att_id, i, points.data() + 3 * i.value());
        pcb.SetAttributeValueForPoint(color_att_id, i, colors.data() + colors_channel * i.value());
      }
    } else {
      for (draco::PointIndex i(0); i < num_points; i++) {
        pcb.SetAttributeValueForPoint(pos_att_id, i, points.data() + 3 * i.value());
      }
    }

    std::unique_ptr<draco::PointCloud> ptr_point_cloud = pcb.Finalize(!preserve_order);
    draco::PointCloud *point_cloud = ptr_point_cloud.get();
    draco::Encoder encoder;
    setup_encoder_and_metadata(point_cloud, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    if (preserve_order) {
      encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }

    draco::EncoderBuffer buffer;
    const draco::Status status = encoder.EncodePointCloudToBuffer(*point_cloud, &buffer);

    EncodedObject encodedPointCloudObject;
    encodedPointCloudObject.buffer = *((std::vector<unsigned char> *)buffer.buffer());
    if (status.ok()) {
      encodedPointCloudObject.encode_status = successful_encoding;
    }
    else {
      std::cerr << "Draco encoding error: " << status.error_msg_string() << std::endl;
      encodedPointCloudObject.encode_status = failed_during_encoding;
    }

    return encodedPointCloudObject;
  }

};

#undef CHECK_STATUS
#endif
