#include <vector>
#include <cstddef>
#include <string>
#include <sstream>

#include "draco/compression/decode.h"
#include "draco/compression/encode.h"
#include "draco/core/encoder_buffer.h"
#include "draco/core/vector_d.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/point_cloud/point_cloud_builder.h"

namespace DracoFunctions {
  enum decoding_status { successful, not_draco_encoded, no_position_attribute, failed_during_decoding };
  enum encoding_status { successful_encoding, failed_during_encoding };

  struct MetadataObject {
    std::unordered_map<std::string, std::string> entries;
    std::unordered_map<std::string, uint32_t> sub_metadata_ids;

    // MetadataObject() = default;
    // MetadataObject(const MetadataObject&) = delete;
  };

  // struct AttributeMetadataObject {
  //   uint32_t metadata_id; // reference to metadata object
  //   uint32_t unique_id;
  // };

  struct PointAttributeObject {
    std::unordered_map<uint32_t, std::string> data;
    draco::DataType datatype;
    uint32_t dimension;
    uint32_t metadata_id;
  };

  struct GeometryMetadataObject {
    uint32_t metadata_id; // reference to metadata object
    std::vector<PointAttributeObject> generic_attributes;
  };

  struct PointCloudObject {
    std::vector<float> points;

    // Encoding options stored in metadata
    bool encoding_options_set;
    int quantization_bits;
    double quantization_range;
    std::vector<double> quantization_origin;

    decoding_status decode_status;

    // so far Geometry
    GeometryMetadataObject geometry_metadata;
    // contains 
    std::vector<MetadataObject> metadatas;
  };

  struct MeshObject : PointCloudObject {
    std::vector<float> normals;
    std::vector<unsigned int> faces;
  };

  struct EncodedObject {
    std::vector<unsigned char> buffer;
    encoding_status encode_status;
  };

  namespace {
    ////////////////////////////////////////////////////
    ////// helpful decoding functions //////////////////
    ////////////////////////////////////////////////////

    uint32_t add_metadata_object(std::vector<MetadataObject>& metadata_objects) {
      metadata_objects.push_back({});
      const auto metadata_id = static_cast<uint32_t>(metadata_objects.size() - 1);
      return metadata_id;
    }
    
    uint32_t decode_metadata(const draco::Metadata& in_metadata, std::vector<MetadataObject>& all_metadata_objects) {
      // main metadata object is a have just created object
      uint32_t metadata_id = add_metadata_object(all_metadata_objects);
      using MetadataPair = std::pair<const draco::Metadata*, uint32_t>;
      for (std::vector<MetadataPair> to_parse_metadata = { {&in_metadata, metadata_id} },
                                     to_parse_metadatas_next;
            !to_parse_metadata.empty();
            to_parse_metadata = std::move(to_parse_metadatas_next)) {
        for (auto& [metadata, metadata_object_idx]: to_parse_metadata) {
          // consider entries
          for (const auto& [name, vec_value]: metadata->entries()) {
            auto raw_value = reinterpret_cast<const char*>(vec_value.data().data());
            auto value_size = vec_value.data().size();
            std::string str_value(raw_value, raw_value + value_size);
            all_metadata_objects[metadata_object_idx].entries[name] = std::move(str_value);
          }
          // consider sub metadatas
          for (const auto& [name, sub_metadata]: metadata->sub_metadatas()) {
            const uint32_t sub_metadata_id = add_metadata_object(all_metadata_objects);
            all_metadata_objects[metadata_object_idx].sub_metadata_ids[name] = sub_metadata_id;
            to_parse_metadatas_next.push_back({sub_metadata.get(), sub_metadata_id});
          }
        }
      
      }
      return metadata_id;
    }

    std::vector<PointAttributeObject> decode_generic_attributes(const draco::PointCloud& pc,
                                                                std::vector<MetadataObject>& all_metadata_objects) {
      std::vector<PointAttributeObject> attribute_objects;
      for (int i = 0; i < pc.num_attributes(); ++i)
        if (const auto* attribute = pc.attribute(i))
          if (attribute->attribute_type() == draco::GeometryAttribute::GENERIC) {
            const uint32_t attribute_id = pc.GetAttributeIdByUniqueId(attribute->unique_id());
            const auto* attribute_metadata = pc.GetAttributeMetadataByAttributeId(attribute_id);
            attribute_metadata = pc.GetAttributeMetadataByAttributeId(attribute->unique_id());
            // decode fields
            PointAttributeObject pao;
            pao.dimension = static_cast<uint32_t>(attribute->num_components());
            pao.datatype = attribute->data_type();
            if (attribute_metadata)
              pao.metadata_id = decode_metadata(*attribute_metadata, all_metadata_objects);
            else
              pao.metadata_id = -1;
            // decode geometry data
            const auto value_size = attribute->num_components() * attribute->byte_stride();
            for (draco::PointIndex v(0); v < attribute->indices_map_size(); ++v) {
              auto& value = pao.data[v.value()];
              value.resize(value_size);
              attribute->GetMappedValue(v, value.data());
            }
            attribute_objects.push_back(std::move(pao));
          }
      return attribute_objects;
    }

    void decode_geometry_metadata(draco::PointCloud& pc,
                                  const draco::GeometryMetadata& geometry_metadata,
                                  PointCloudObject& pco) {
      pco.geometry_metadata.metadata_id = decode_metadata(geometry_metadata, pco.metadatas);
      pco.geometry_metadata.generic_attributes = decode_generic_attributes(pc, pco.metadatas);
    }

    ////////////////////////////////////////////////////
    ////// helpful encoding functions //////////////////
    ////////////////////////////////////////////////////

    int32_t encode_generic_attribute(
      const PointAttributeObject& pao,
      draco::PointCloud& pc
    ) {
      draco::GeometryAttribute base_attribute;
      base_attribute.Init(draco::GeometryAttribute::GENERIC, nullptr, 
                          pao.dimension, pao.datatype, false, 
                          draco::DataTypeLength(pao.datatype) * pao.dimension, 0);
      int32_t attribute_id = pc.AddAttribute(base_attribute, false, pao.data.size());
      if (attribute_id < 0)
        return attribute_id;
      draco::PointAttribute* attribute = pc.attribute(attribute_id);
      for (draco::PointIndex v(0); v < pao.data.size(); ++v) {
        draco::AttributeValueIndex value_index(v.value());
        attribute->SetPointMapEntry(v, value_index);
        try {
          const auto& point_value = pao.data.at(v.value());
          attribute->SetAttributeValue(value_index, point_value.data());
        } catch (std::out_of_range& e ) {
          std::cout << e.what() << std::endl;
        }
      }
      return attribute_id;
    }

    void encode_metadata(const std::vector<MetadataObject>& metadatas,
                         const MetadataObject& main_metadata_object,
                         draco::Metadata& out_metadata) {
      // it should not encode metadata as it doesn't exist
      if (metadatas.empty())
        return;
      using MetadataPair = std::pair<draco::Metadata*, const MetadataObject*>;
      for (std::vector<MetadataPair> to_parse_metadata = { {&out_metadata, &main_metadata_object} }, 
                                     to_parse_metadatas_next;
            !to_parse_metadata.empty(); 
            to_parse_metadata = std::move(to_parse_metadatas_next)) {
        for (auto& [metadata, metadata_object]: to_parse_metadata) {
          // consider entries
          for (const auto& [name, str_value]: metadata_object->entries) {
            std::vector<uint8_t> vec_value(str_value.size());
            memcpy(vec_value.data(), str_value.data(), str_value.size());
            metadata->AddEntryBinary(name, vec_value);
          }
          // consider sub metadatas
          for (const auto& [name, metadata_id]: metadata_object->sub_metadata_ids) {
            auto sub_metadata = std::make_unique<draco::Metadata>();
            to_parse_metadatas_next.push_back({sub_metadata.get(), &metadatas[metadata_id]});
            metadata->AddSubMetadata(name, std::move(sub_metadata));
          }
        }
        
      }
    }
  
    bool encode_geometry_metadata(
            draco::PointCloud& pc,
            const std::vector<MetadataObject>& metadatas,
            const GeometryMetadataObject& geometry_metadata_object) {
      // it should not encode geometry metadata if actually no metadata was passed
      if (metadatas.empty())
        return false;
      const auto& gmo = geometry_metadata_object;
      auto geometry_metadata_ptr = std::make_unique<draco::GeometryMetadata>();
      auto& geometry_metadata = *geometry_metadata_ptr;
      pc.AddMetadata(std::move(geometry_metadata_ptr));
      encode_metadata(metadatas, metadatas[gmo.metadata_id], geometry_metadata);
      for (const auto& attribute_object: gmo.generic_attributes) {
        int32_t attribute_id = encode_generic_attribute(attribute_object, pc);
        if (attribute_id < 0)
          continue;
        auto attribute_metadata = std::make_unique<draco::AttributeMetadata>();
        encode_metadata(metadatas, metadatas[attribute_object.metadata_id], *attribute_metadata);

        auto& attribute_metadata_ref = *attribute_metadata;
        pc.AddAttributeMetadata(attribute_id, std::move(attribute_metadata));
      }
      return true;
    }
  }

  MeshObject decode_buffer(const char *buffer, std::size_t buffer_len, bool
  deduplicate) {
    MeshObject meshObject;
    draco::DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);
    draco::Decoder decoder;
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
    std::unique_ptr<draco::Mesh> in_mesh = std::move(statusor).value();
    draco::Mesh *mesh = in_mesh.get();
    if (deduplicate) {
      mesh->DeduplicateAttributeValues();
      mesh->DeduplicatePointIds();
    }
    const int pos_att_id = mesh->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      meshObject.decode_status = no_position_attribute;
      return meshObject;
    }
    meshObject.points.reserve(3 * mesh->num_points());
    meshObject.faces.reserve(3 * mesh->num_faces());
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
    for (draco::FaceIndex i(0); i < mesh->num_faces(); ++i) {
      const auto &f = mesh->face(i);
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[0]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[1]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[2]))));
    }
    const draco::GeometryMetadata *metadata = mesh->GetMetadata();
    meshObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(meshObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(meshObject.quantization_range)) &&
          metadata->GetEntryDoubleArray("quantization_origin", &(meshObject.quantization_origin))) {
          meshObject.encoding_options_set = true;
      }
      decode_geometry_metadata(*mesh, *metadata, meshObject);
    }
    meshObject.decode_status = successful;
    return meshObject;
  }

  PointCloudObject decode_buffer_to_point_cloud(const char *buffer, std::size_t
  buffer_len, bool deduplicate) {
    PointCloudObject pointCloudObject;
    draco::DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);
    draco::Decoder decoder;
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
    std::unique_ptr<draco::PointCloud> in_point_cloud = std::move(statusor).value();
    draco::PointCloud *point_cloud = in_point_cloud.get();
    if (deduplicate) {
      point_cloud->DeduplicateAttributeValues();
      point_cloud->DeduplicatePointIds();
    }
    const int pos_att_id = point_cloud->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      pointCloudObject.decode_status = no_position_attribute;
      return pointCloudObject;
    }
    pointCloudObject.points.reserve(3 * point_cloud->num_points());
    const auto *const pos_att = point_cloud->attribute(pos_att_id);
    std::array<float, 3> pos_val;
    for (draco::PointIndex v(0); v < point_cloud->num_points(); ++v) {
      if (!pos_att->ConvertValue<float, 3>(pos_att->mapped_index(v), &pos_val[0])) {
        pointCloudObject.decode_status = no_position_attribute;
        return pointCloudObject;
      }
      pointCloudObject.points.push_back(pos_val[0]);
      pointCloudObject.points.push_back(pos_val[1]);
      pointCloudObject.points.push_back(pos_val[2]);
    }
    const draco::GeometryMetadata *metadata = point_cloud->GetMetadata();
    pointCloudObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(pointCloudObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(pointCloudObject.quantization_range)) &&
        metadata->GetEntryDoubleArray("quantization_origin", &(pointCloudObject.quantization_origin))) {
        pointCloudObject.encoding_options_set = true;
      }
      decode_geometry_metadata(*point_cloud, *metadata, pointCloudObject);
    }
    pointCloudObject.decode_status = successful;
    return pointCloudObject;
  }

  void setup_encoder_and_metadata(draco::PointCloud *point_cloud_or_mesh, draco::Encoder &encoder, int compression_level, int quantization_bits, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int speed = 10 - compression_level;
    encoder.SetSpeedOptions(speed, speed);
    auto metadata = std::make_unique<draco::GeometryMetadata>();
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

  EncodedObject encode_mesh(const std::vector<float> &points, 
                            const std::vector<unsigned int> &faces,
                            const std::vector<MetadataObject>& metadatas,
                            const GeometryMetadataObject& geometry_metadata_object,
                            
                            int quantization_bits, int compression_level, float quantization_range,
                            const float *quantization_origin, bool create_metadata) {
    draco::TriangleSoupMeshBuilder mb;
    mb.Start(faces.size());
    const int pos_att_id =
    mb.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DataType::DT_FLOAT32);
    for (std::size_t i = 0; i <= faces.size() - 3; i += 3) {
      auto point1Index = faces[i]*3;
      auto point2Index = faces[i+1]*3;
      auto point3Index = faces[i+2]*3;
      mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i), 
                                   draco::Vector3f(points[point1Index], points[point1Index+1], points[point1Index+2]).data(),
                                   draco::Vector3f(points[point2Index], points[point2Index+1], points[point2Index+2]).data(),
                                   draco::Vector3f(points[point3Index], points[point3Index+1], points[point3Index+2]).data());
    }

    std::unique_ptr<draco::Mesh> ptr_mesh = mb.Finalize();
    draco::Mesh *mesh = ptr_mesh.get();
    
    encode_geometry_metadata(*mesh, metadatas, geometry_metadata_object);
    

    draco::Encoder encoder;
    setup_encoder_and_metadata(mesh, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    draco::EncoderBuffer buffer;
    const draco::Status status = encoder.EncodeMeshToBuffer(*mesh, &buffer);
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

  EncodedObject encode_point_cloud(const std::vector<float> &points, const PointCloudObject& pco,
                                   int quantization_bits, int compression_level,
                                   float quantization_range, const float *quantization_origin,
                                    bool create_metadata) {
    int num_points = points.size() / 3;
    draco::PointCloudBuilder pcb;
    pcb.Start(num_points);
    const int pos_att_id =
      pcb.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DataType::DT_FLOAT32);

    for (draco::PointIndex i(0); i < num_points; i++) {
      pcb.SetAttributeValueForPoint(pos_att_id, i, points.data() + 3 * i.value());  
    }

    std::unique_ptr<draco::PointCloud> ptr_point_cloud = pcb.Finalize(true);
    draco::PointCloud *point_cloud = ptr_point_cloud.get();
    encode_geometry_metadata(*point_cloud, pco.metadatas, pco.geometry_metadata);

    draco::Encoder encoder;
    setup_encoder_and_metadata(point_cloud, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    draco::EncoderBuffer buffer;
    const draco::Status status = encoder.EncodePointCloudToBuffer(*point_cloud, &buffer);
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
