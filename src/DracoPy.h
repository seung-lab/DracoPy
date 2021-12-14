#ifndef __DRACOPY_H__
#define __DRACOPY_H__

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
    no_normal_coord_attribute,
    failed_during_decoding 
  };
  enum encoding_status { 
    successful_encoding, 
    failed_during_encoding 
  };

  struct PointCloudObject {
    std::vector<float> points;

    // Encoding options stored in metadata
    bool encoding_options_set;
    int quantization_bits;
    double quantization_range;
    std::vector<double> quantization_origin;

    decoding_status decode_status;
  };

  struct MeshObject : PointCloudObject {
    std::vector<float> normals;
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

    const int pos_att_id = mesh->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      meshObject.decode_status = no_position_attribute;
      return meshObject;
    }

    meshObject.points.reserve(3 * mesh->num_points());
    const auto *const pos_att = mesh->attribute(pos_att_id);
    std::array<float, 3> pos_val;
    for (draco::PointIndex v(0); v < mesh->num_points(); ++v) {
      if (!pos_att->ConvertValue<float, 3>(pos_att->mapped_index(v), &pos_val[0])) {
        meshObject.decode_status = no_position_attribute;
        return meshObject;
      }
      meshObject.points.push_back(pos_val[0]);
      meshObject.points.push_back(pos_val[1]);
      meshObject.points.push_back(pos_val[2]);
    }

    const draco::GeometryMetadata *metadata = mesh->GetMetadata();
    meshObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(meshObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(meshObject.quantization_range)) &&
          metadata->GetEntryDoubleArray("quantization_origin", &(meshObject.quantization_origin))) {
          meshObject.encoding_options_set = true;
      }
    }

    if (geotype == draco::EncodedGeometryType::POINT_CLOUD) {
      meshObject.decode_status = successful;
      return meshObject;
    }

    meshObject.faces.reserve(3 * mesh->num_faces());
    for (draco::FaceIndex i(0); i < mesh->num_faces(); ++i) {
      const auto &f = mesh->face(i);
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[0]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[1]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[2]))));
    }

    const int normal_att_id = mesh->GetNamedAttributeId(draco::GeometryAttribute::NORMAL);
    if (normal_att_id < 0) {  // No normal values are present.
      meshObject.decode_status = successful;
      return meshObject;
    }

    const auto *const normal_att = mesh->attribute(normal_att_id);
    meshObject.normals.reserve(3 * normal_att->size());

    std::array<float, 3> normal_val;
    for (draco::PointIndex v(0); v < normal_att->size(); ++v){
      if (!normal_att->ConvertValue<float, 3>(normal_att->mapped_index(v), &normal_val[0])){
        meshObject.decode_status = no_normal_coord_attribute;
      }
      meshObject.normals.push_back(normal_val[0]);
      meshObject.normals.push_back(normal_val[1]);
      meshObject.normals.push_back(normal_val[3]);
    }

    meshObject.decode_status = successful;
    return meshObject;
  }

  void setup_encoder_and_metadata(draco::PointCloud *point_cloud_or_mesh, draco::Encoder &encoder, int compression_level, int quantization_bits, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int speed = 10 - compression_level;
    encoder.SetSpeedOptions(speed, speed);
    std::unique_ptr<draco::GeometryMetadata> metadata = std::unique_ptr<draco::GeometryMetadata>(new draco::GeometryMetadata());
    if (quantization_origin == NULL || quantization_range == -1) {
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
    const bool create_metadata,
    const bool integer_positions = false
  ) {
    draco::TriangleSoupMeshBuilder mb;
    mb.Start(faces.size());

    auto dtype = integer_positions 
      ? draco::DataType::DT_UINT32
      : draco::DataType::DT_FLOAT32;

    const int pos_att_id = mb.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, dtype
    );

    if (integer_positions) {
      for (std::size_t i = 0; i <= faces.size() - 3; i += 3) {
        auto p1 = faces[i]*3;
        auto p2 = faces[i+1]*3;
        auto p3 = faces[i+2]*3;

        mb.SetAttributeValuesForFace(
          pos_att_id, draco::FaceIndex(i), 
          draco::Vector3ui(points[p1], points[p1+1], points[p1+2]).data(), 
          draco::Vector3ui(points[p2], points[p2+1], points[p2+2]).data(), 
          draco::Vector3ui(points[p3], points[p3+1], points[p3+2]).data()
        );
      }
    }
    else {
      for (std::size_t i = 0; i <= faces.size() - 3; i += 3) {
        auto p1 = faces[i]*3;
        auto p2 = faces[i+1]*3;
        auto p3 = faces[i+2]*3;

        mb.SetAttributeValuesForFace(
          pos_att_id, draco::FaceIndex(i), 
          draco::Vector3f(points[p1], points[p1+1], points[p1+2]).data(), 
          draco::Vector3f(points[p2], points[p2+1], points[p2+2]).data(), 
          draco::Vector3f(points[p3], points[p3+1], points[p3+2]).data()
        );
      }
    }

    std::unique_ptr<draco::Mesh> ptr_mesh = mb.Finalize();
    draco::Mesh *mesh = ptr_mesh.get();
    draco::Encoder encoder;
    setup_encoder_and_metadata(
      mesh, encoder, compression_level, 
      quantization_bits, quantization_range, 
      quantization_origin, create_metadata
    );
  

    draco::EncoderBuffer buffer;
    const draco::Status status = encoder.EncodeMeshToBuffer(*mesh, &buffer);
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
    const float *quantization_origin, const bool create_metadata,
    const bool integer_positions = false
  ) {
    int num_points = points.size() / 3;
    draco::PointCloudBuilder pcb;
    pcb.Start(num_points);

    auto dtype = integer_positions 
      ? draco::DataType::DT_UINT32
      : draco::DataType::DT_FLOAT32;

    const int pos_att_id = pcb.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, dtype
    );

    for (draco::PointIndex i(0); i < num_points; i++) {
      pcb.SetAttributeValueForPoint(pos_att_id, i, points.data() + 3 * i.value());  
    }

    std::unique_ptr<draco::PointCloud> ptr_point_cloud = pcb.Finalize(true);
    draco::PointCloud *point_cloud = ptr_point_cloud.get();
    draco::Encoder encoder;
    setup_encoder_and_metadata(point_cloud, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    
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
