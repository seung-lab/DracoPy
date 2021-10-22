#include<vector>
#include<cstddef>
#include <string>
#include <sstream>

#include "draco/compression/decode.h"
#include "draco/compression/encode.h"
#include "draco/core/encoder_buffer.h"
#include "draco/core/vector_d.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/point_cloud/point_cloud_builder.h"

namespace DracoFunctions {
    using namespace draco;

  enum decoding_status { successful, not_draco_encoded, no_position_attribute, failed_during_decoding };
  enum encoding_status { successful_encoding, failed_during_encoding };

  struct MetadataObject {
    uint32_t tree_id;  // id to help build tree structure
    std::unordered_map<std::string, std::string> entries;
    std::unordered_map<std::string, uint32_t> sub_metadata_ids;
  };

  struct AttributeMetadataObject: MetadataObject {
    uint32_t unique_id;
  };

  struct PointAttributeObject {
    std::unordered_map<uint32_t, std::string> data;
    uint32_t element_size;
    uint32_t dimension;
    uint32_t unique_id;
  };

  struct GeometryMetadataObject: MetadataObject {
    std::vector<AttributeMetadataObject> attribute_metadatas;
  };

  struct PointCloudObject {
    std::vector<float> points;

    // Encoding options stored in metadata
    bool encoding_options_set;
    int quantization_bits;
    double quantization_range;
    std::vector<double> quantization_origin;

    decoding_status decode_status;

    std::vector<PointAttributeObject> attributes;
    GeometryMetadataObject geometry_metadata;
  };

  struct MeshObject : PointCloudObject {
    std::vector<float> normals;
    std::vector<unsigned int> faces;
  };

  struct EncodedObject {
    std::vector<unsigned char> buffer;
    encoding_status encode_status;
  };

  std::vector<PointAttributeObject> decode_attributes(const PointCloud& pc) {
    const int32_t attributes_len = pc.num_attributes();
    std::vector<PointAttributeObject> attribute_objects(attributes_len);
    for (int i = 0; i < attributes_len; ++i)
      if (const auto* attribute = pc.attribute(i))
      {
        attribute_objects[i].dimension = static_cast<uint32_t>(attribute->num_components());
        attribute_objects[i].element_size = static_cast<uint32_t>(attribute->byte_stride());
        attribute_objects[i].unique_id = static_cast<uint32_t>(attribute->unique_id());
        const auto value_size = attribute->num_components() * attribute->byte_stride();
        for (PointIndex v(0); v < attribute->indices_map_size(); ++v) {
          auto& value = attribute_objects[i].data[v.value()];
          value.resize(value_size);
          attribute->GetMappedValue(v, value.data());
        }
      }
    return attribute_objects;
  }

  void decode_metadata(const Metadata& metadata, MetadataObject& main_metadata_object) {
    main_metadata_object.tree_id = 0;
    using MetadataPair = std::pair<const Metadata*, MetadataObject*>;
    std::vector<MetadataPair> to_parse_metadata = { {&metadata, &main_metadata_object} }; 
    std::vector<MetadataObject> metadata_objects;
    uint32_t metadata_idx = 0;
    for (std::vector<MetadataPair> to_parse_metadatas_next; !to_parse_metadata.empty(); 
                to_parse_metadata = std::move(to_parse_metadatas_next)) {
      for (auto [metadata, metadata_object]: to_parse_metadata) {
        // consider entries
        for (const auto& [name, vec_value]: metadata->entries()) {
          auto raw_value = reinterpret_cast<const char*>(vec_value.data().data());
          auto value_size = vec_value.data().size();
          std::string str_value(raw_value, raw_value + value_size);
          metadata_object->entries[name] = std::move(str_value);
        }
        // consider sub metadatas
        for (const auto& [name, sub_metadata]: metadata->sub_metadatas()) {
            MetadataObject sub_metadata_object;
            sub_metadata_object.tree_id = ++metadata_idx;
            metadata_object->sub_metadata_ids[name] = sub_metadata_object.tree_id;
            metadata_objects.push_back(std::move(sub_metadata_object));
            to_parse_metadatas_next.push_back({sub_metadata.get(), &sub_metadata_object});
        }
      }
    }
  }

  GeometryMetadataObject decode_geometry_metadata(const GeometryMetadata& geometry_metadata) {
    GeometryMetadataObject geometry_metadata_object;
    decode_metadata(static_cast<const Metadata&>(geometry_metadata),
                    static_cast<MetadataObject&>(geometry_metadata_object));
    for (const auto& attribute_metadata: geometry_metadata.attribute_metadatas()) {
      AttributeMetadataObject attribute_metadata_object;
      attribute_metadata_object.unique_id = attribute_metadata->att_unique_id();
      decode_metadata(static_cast<const Metadata&>(*attribute_metadata.get()),
                      static_cast<MetadataObject&>(attribute_metadata_object));
      geometry_metadata_object.attribute_metadatas.push_back(std::move(attribute_metadata_object));
    }
    return geometry_metadata_object;
  }

  MeshObject decode_buffer(const char *buffer, std::size_t buffer_len) {
    MeshObject meshObject;
    DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);
    Decoder decoder;
    auto statusor = decoder.DecodeMeshFromBuffer(&decoderBuffer);
    if (!statusor.ok()) {
      std::string status_string = statusor.status().error_msg_string();
      if (status_string.compare("Not a Draco file.") || status_string.compare("Failed to parse Draco header.")) {
        meshObject.decode_status = not_draco_encoded;
      }
      else {
        meshObject.decode_status = failed_during_decoding;
      }
      return meshObject;
    }
    std::unique_ptr<Mesh> in_mesh = std::move(statusor).value();
    Mesh *mesh = in_mesh.get();
    const int pos_att_id = mesh->GetNamedAttributeId(GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      meshObject.decode_status = no_position_attribute;
      return meshObject;
    }
    meshObject.points.reserve(3 * mesh->num_points());
    meshObject.faces.reserve(3 * mesh->num_faces());
    const auto *const pos_att = mesh->attribute(pos_att_id);
    std::array<float, 3> pos_val;
    for (PointIndex v(0); v < mesh->num_points(); ++v) {
      if (!pos_att->ConvertValue<float, 3>(pos_att->mapped_index(v), &pos_val[0])) {
        meshObject.decode_status = no_position_attribute;
        return meshObject;
      }
      meshObject.points.push_back(pos_val[0]);
      meshObject.points.push_back(pos_val[1]);
      meshObject.points.push_back(pos_val[2]);
    }
    for (FaceIndex i(0); i < mesh->num_faces(); ++i) {
      const auto &f = mesh->face(i);
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[0]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[1]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[2]))));
    }
    const GeometryMetadata *metadata = mesh->GetMetadata();
    meshObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(meshObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(meshObject.quantization_range)) &&
          metadata->GetEntryDoubleArray("quantization_origin", &(meshObject.quantization_origin))) {
          meshObject.encoding_options_set = true;
      }
      meshObject.geometry_metadata = decode_geometry_metadata(*metadata);
    }
    meshObject.attributes = decode_attributes(*mesh);
    meshObject.decode_status = successful;
    return meshObject;
  }

  PointCloudObject decode_buffer_to_point_cloud(const char *buffer, std::size_t buffer_len) {
    PointCloudObject pointCloudObject;
    DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);
    Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&decoderBuffer);
    if (!statusor.ok()) {
      std::string status_string = statusor.status().error_msg_string();
      if (status_string.compare("Not a Draco file.") || status_string.compare("Failed to parse Draco header.")) {
        pointCloudObject.decode_status = not_draco_encoded;
      }
      else {
        pointCloudObject.decode_status = failed_during_decoding;
      }
      return pointCloudObject;
    }
    std::unique_ptr<PointCloud> in_point_cloud = std::move(statusor).value();
    PointCloud *point_cloud = in_point_cloud.get();
    const int pos_att_id = point_cloud->GetNamedAttributeId(GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      pointCloudObject.decode_status = no_position_attribute;
      return pointCloudObject;
    }
    pointCloudObject.points.reserve(3 * point_cloud->num_points());
    const auto *const pos_att = point_cloud->attribute(pos_att_id);
    std::array<float, 3> pos_val;
    for (PointIndex v(0); v < point_cloud->num_points(); ++v) {
      if (!pos_att->ConvertValue<float, 3>(pos_att->mapped_index(v), &pos_val[0])) {
        pointCloudObject.decode_status = no_position_attribute;
        return pointCloudObject;
      }
      pointCloudObject.points.push_back(pos_val[0]);
      pointCloudObject.points.push_back(pos_val[1]);
      pointCloudObject.points.push_back(pos_val[2]);
    }
    const GeometryMetadata *metadata = point_cloud->GetMetadata();
    pointCloudObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(pointCloudObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(pointCloudObject.quantization_range)) &&
        metadata->GetEntryDoubleArray("quantization_origin", &(pointCloudObject.quantization_origin))) {
        pointCloudObject.encoding_options_set = true;
      }
      pointCloudObject.geometry_metadata = decode_geometry_metadata(*metadata);
    }
    pointCloudObject.attributes = decode_attributes(*point_cloud);
    pointCloudObject.decode_status = successful;
    return pointCloudObject;
  }

  void setup_encoder_and_metadata(PointCloud *point_cloud_or_mesh, Encoder &encoder, int compression_level, int quantization_bits, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int speed = 10 - compression_level;
    encoder.SetSpeedOptions(speed, speed);
    std::unique_ptr<GeometryMetadata> metadata = std::unique_ptr<GeometryMetadata>(new GeometryMetadata());
    if (quantization_origin == NULL || quantization_range == -1) {
      encoder.SetAttributeQuantization(GeometryAttribute::POSITION, quantization_bits);
    } 
    else {
      encoder.SetAttributeExplicitQuantization(GeometryAttribute::POSITION, quantization_bits, 3, quantization_origin, quantization_range);
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

  EncodedObject encode_mesh(const std::vector<float> &points, const 
  std::vector<unsigned int> &faces,
      int quantization_bits, int compression_level, float quantization_range, const float *quantization_origin, bool create_metadata) {
    TriangleSoupMeshBuilder mb;
    mb.Start(faces.size());
    const int pos_att_id =
      mb.AddAttribute(GeometryAttribute::POSITION, 3, DataType::DT_FLOAT32);

    for (std::size_t i = 0; i <= faces.size() - 3; i += 3) {
      auto point1Index = faces[i]*3;
      auto point2Index = faces[i+1]*3;
      auto point3Index = faces[i+2]*3;
      mb.SetAttributeValuesForFace(pos_att_id, FaceIndex(i), Vector3f(points[point1Index], points[point1Index+1], points[point1Index+2]).data(), Vector3f(points[point2Index], points[point2Index+1], points[point2Index+2]).data(), Vector3f(points[point3Index], points[point3Index+1], points[point3Index+2]).data());
    }

    std::unique_ptr<Mesh> ptr_mesh = mb.Finalize();
    Mesh *mesh = ptr_mesh.get();
    Encoder encoder;
    setup_encoder_and_metadata(mesh, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    EncoderBuffer buffer;
    const Status status = encoder.EncodeMeshToBuffer(*mesh, &buffer);
    EncodedObject encodedMeshObject;
    encodedMeshObject.buffer = *((std::vector<unsigned char> *)buffer.buffer());
    if (status.ok()) {
      encodedMeshObject.encode_status = successful_encoding;
    } else {
      std::cout << "Draco encoding error: " << status.error_msg_string() << std::endl;
      encodedMeshObject.encode_status = failed_during_encoding;
    }
    return encodedMeshObject;
  }

  EncodedObject encode_point_cloud(const std::vector<float> &points, int quantization_bits,
      int compression_level, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int num_points = points.size() / 3;
    PointCloudBuilder pcb;
    pcb.Start(num_points);
    const int pos_att_id =
      pcb.AddAttribute(GeometryAttribute::POSITION, 3, DataType::DT_FLOAT32);

    for (PointIndex i(0); i < num_points; i++) {
      pcb.SetAttributeValueForPoint(pos_att_id, i, points.data() + 3 * i.value());  
    }

    std::unique_ptr<PointCloud> ptr_point_cloud = pcb.Finalize(true);
    PointCloud *point_cloud = ptr_point_cloud.get();
    Encoder encoder;
    setup_encoder_and_metadata(point_cloud, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    EncoderBuffer buffer;
    const Status status = encoder.EncodePointCloudToBuffer(*point_cloud, &buffer);
    EncodedObject encodedPointCloudObject;
    encodedPointCloudObject.buffer = *((std::vector<unsigned char> *)buffer.buffer());
    if (status.ok()) {
      encodedPointCloudObject.encode_status = successful_encoding;
    } else {
      std::cout << "Draco encoding error: " << status.error_msg_string() << std::endl;
      encodedPointCloudObject.encode_status = failed_during_encoding;
    }
    return encodedPointCloudObject;
  }
}
