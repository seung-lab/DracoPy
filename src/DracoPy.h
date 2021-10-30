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
      std::cout << 'a' << std::endl;
      // main metadata object is a have just created object
      uint32_t metadata_id = add_metadata_object(all_metadata_objects);
      MetadataObject& main_metadata_object = all_metadata_objects.back();

      std::cout << 'b' << std::endl;
      using MetadataPair = std::pair<const draco::Metadata*, MetadataObject*>;
      std::cout << 'c' << std::endl;
      std::vector<MetadataPair> to_parse_metadata = { {&in_metadata, &main_metadata_object} }; 
      std::cout << 'd' << std::endl;
      for (std::vector<MetadataPair> to_parse_metadatas_next; !to_parse_metadata.empty(); 
                  ) {
        std::cout << 'e' << std::endl;
        for (auto& [metadata, metadata_object]: to_parse_metadata) {
          std::cout << 'f' << std::endl;
          std::cout << "metadata " <<  metadata << std::endl;
          std::cout << "metadata_object " <<  metadata_object << std::endl;
          // consider entries
          for (const auto& [name, vec_value]: metadata->entries()) {
            std::cout << 'e' << std::endl;
            auto raw_value = reinterpret_cast<const char*>(vec_value.data().data());
            std::cout << 'h' << std::endl;
            auto value_size = vec_value.data().size();
            std::cout << 'i' << std::endl;
            std::string str_value(raw_value, raw_value + value_size);
            std::cout << 'j' << std::endl;
            metadata_object->entries[name] = std::move(str_value);
          }
          // consider sub metadatas
          for (const auto& [name, sub_metadata]: metadata->sub_metadatas()) {
            std::cout << 'k' << std::endl;
            std::cout << name << std::endl;
            std::cout << metadata_object->entries.size() << std::endl;
            std::cout << metadata_object->sub_metadata_ids.size() << std::endl;
            const uint32_t sub_metadata_id = add_metadata_object(all_metadata_objects);
            std::cout << "sub metadata id " << sub_metadata_id << std::endl;
            metadata_object->sub_metadata_ids.reserve(30);
            std::cout << "reserved " << sub_metadata_id << std::endl;
            metadata_object->sub_metadata_ids[name] = sub_metadata_id;
            std::cout << metadata_object->sub_metadata_ids.size() << std::endl;
            std::cout << 'l' << std::endl;
            to_parse_metadatas_next.push_back({sub_metadata.get(), &all_metadata_objects.back()});
            std::cout << 'm' << std::endl;
          }
        }
      to_parse_metadata = std::move(to_parse_metadatas_next);
      }
      return metadata_id;
    }

    std::vector<PointAttributeObject> decode_generic_attributes(const draco::PointCloud& pc,
                                                                std::vector<MetadataObject>& all_metadata_objects) {
      std::vector<PointAttributeObject> attribute_objects;
      for (int i = 0; i < pc.num_attributes(); ++i)
        if (const auto* attribute = pc.attribute(i))
          if (attribute->attribute_type() == draco::GeometryAttribute::GENERIC) {
            std::cout << "GetAttributeIdByUniqueId" << std::endl;
            const uint32_t attribute_id = pc.GetAttributeIdByUniqueId(attribute->unique_id());
            // pc.metadata()->)
            std::cout << "attribute_id " << attribute_id << std::endl;
            std::cout << "attribute->unique_id() " << attribute->unique_id() << std::endl;
            std::cout << "GetAttributeMetadataByAttributeId" << std::endl;
            const auto* attribute_metadata = pc.GetAttributeMetadataByAttributeId(attribute_id);
            std::cout << "attribute_metadata " << attribute_metadata << std::endl;
            attribute_metadata = pc.GetAttributeMetadataByAttributeId(attribute->unique_id());
            std::cout << "attribute_metadata " << attribute_metadata << std::endl;
            // decode fields
            PointAttributeObject pao;
            pao.dimension = static_cast<uint32_t>(attribute->num_components());
            pao.datatype = attribute->data_type();
            std::cout << "decode_metadata" << std::endl;
            if (attribute_metadata)
              pao.metadata_id = decode_metadata(*attribute_metadata, all_metadata_objects);
            else
              pao.metadata_id = -1;
            std::cout << "attribute->num_components()" << std::endl;
            // decode geometry data
            const auto value_size = attribute->num_components() * attribute->byte_stride();
            std::cout << "for" << std::endl;
            for (draco::PointIndex v(0); v < attribute->indices_map_size(); ++v) {
              std::cout << "value = attribute_objects[i].data" << std::endl;
              auto& value = pao.data[v.value()];
              value.resize(value_size);
              std::cout << "get mapped value" << std::endl;
              attribute->GetMappedValue(v, value.data());
            }
            attribute_objects.push_back(std::move(pao));
          }
      std::cout << "return from decode_generic_attributes" << std::endl;
      return attribute_objects;
    }

    void decode_geometry_metadata(draco::PointCloud& pc,
                                  const draco::GeometryMetadata& geometry_metadata,
                                  PointCloudObject& pco) {
      std::cout << "decode_metadata" << std::endl;
      pco.geometry_metadata.metadata_id = decode_metadata(geometry_metadata, pco.metadatas);
      std::cout << "decode_generic_attributes" << std::endl;
      pco.geometry_metadata.generic_attributes = decode_generic_attributes(pc, pco.metadatas);

      // // geometry_metadata
      // for (const auto& attriute_metadata: geometry_metadata.attribute_metadatas()) {
      //   auto metadata_id = decode_metadata(*attriute_metadata, pco.metadatas);
      //   std::cout << "not found att_unique_id = " << attriute_metadata->att_unique_id();
      //   std::cout << "metadata_id = " << metadata_id << std::endl;
      // }
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
      // attribute->SetExplicitMapping(pao.data.size());
      for (draco::PointIndex v(0); v < pao.data.size(); ++v) {
        // std::cout << 5 << std::endl;
        draco::AttributeValueIndex value_index(v.value());
        // std::cout << 6 << std::endl;
        attribute->SetPointMapEntry(v, value_index);
        // std::cout << 7 << std::endl;
        // std::cout << value_index << std::endl;
        // std::cout << pao.data.size() << std::endl;
        // for (const auto item: pao.data) {
        //   std::cout << '(' << item.first << ',';
        //   std::cout << *reinterpret_cast<const uint32_t*>(item.second.data()) << ") ";
        // }
        std::cout << std::endl;
        try {
          const auto& point_value = pao.data.at(v.value());
          std::cout << "point_value " << point_value.size() << std::endl;
          attribute->SetAttributeValue(value_index, point_value.data());
        } catch (std::out_of_range& e ) {
        std::cout << e.what() << std::endl;
        }
        std::cout << 8 << std::endl;
      }
      return attribute_id;
    }

    // void encode_generic_attributes(const std::vector<PointAttributeObject>& attributes,
    //                                draco::PointCloud& pc) {
    //   std::cout << "start" << std::endl;
    //   for (const PointAttributeObject& pao: attributes) {
    //     std::cout << 1 << std::endl;
    //     draco::GeometryAttribute base_attribute;
    //     base_attribute.Init(draco::GeometryAttribute::GENERIC, nullptr, 
    //                         pao.dimension, pao.datatype, false, 
    //                         draco::DataTypeLength(pao.datatype) * pao.dimension, 0);
    //     auto attribute = pc.CreateAttribute(base_attribute, false, pao.data.size());
    //     std::cout << 2 << std::endl;
    //     attribute->set_unique_id(pao.unique_id);
    //     std::cout << 3 << std::endl;
    //     attribute->SetExplicitMapping(pao.data.size());
    //     std::cout << 4 << std::endl;
    //     for (draco::PointIndex v(0); v < pao.data.size(); ++v) {
    //       std::cout << 5 << std::endl;
    //       draco::AttributeValueIndex value_index(v.value());
    //       std::cout << 6 << std::endl;
    //       attribute->SetPointMapEntry(v, value_index);
    //       std::cout << 7 << std::endl;
    //       std::cout << value_index << std::endl;
    //       std::cout << pao.data.size() << std::endl;
    //       for (const auto item: pao.data) {
    //         std::cout << '(' << item.first << ',';
    //         std::cout << *reinterpret_cast<const uint32_t*>(item.second.data()) << ") ";
    //       }
    //       std::cout << std::endl;
    //       try {
    //         const auto& point_value = pao.data.at(v.value());
    //         std::cout << "point_value " << point_value.size() << std::endl;
    //         attribute->SetAttributeValue(value_index, point_value.data());
    //       } catch (std::out_of_range& e ) {
    //       std::cout << e.what() << std::endl;
    //       }
    //       std::cout << 8 << std::endl;
    //     }
    //     std::cout << 10 << std::endl;
    //     pc.AddAttribute(std::move(attribute));
    //     std::cout << 9 << std::endl;
    //   }
    //   std::cout << "finish" << std::endl;
    // }

    void encode_metadata(const std::vector<MetadataObject>& metadatas,
                         const MetadataObject& main_metadata_object,
                         draco::Metadata& out_metadata) {
      // it should not encode geometry metadata as it doesn't exist
      if (metadatas.empty())
        return;
      using MetadataPair = std::pair<draco::Metadata*, const MetadataObject*>;
      std::vector<MetadataPair> to_parse_metadata = { {&out_metadata, &main_metadata_object} }; 
      for (std::vector<MetadataPair> to_parse_metadatas_next; !to_parse_metadata.empty(); 
                  ) {
        // std::cout << "to_parse_metadata begin" << std::endl;
        // for (auto& pair: to_parse_metadata)
        //     std::cout << pair.first << ' ' << pair.second << std::endl;
        for (auto& [metadata, metadata_object]: to_parse_metadata) {
          // std::cout << "4" << std::endl;
          // std::cout << metadata << std::endl;
          // std::cout << metadata_object << std::endl;
          // consider entries
          for (const auto& [name, str_value]: metadata_object->entries) {
            // std::cout << "5 " << name << ' ' << str_value << std::endl;
            std::vector<uint8_t> vec_value(str_value.size());
            // std::cout << "6" << std::endl;
            memcpy(vec_value.data(), str_value.data(), str_value.size());
            // std::cout << "7" << std::endl;
            metadata->AddEntryBinary(name, vec_value);
            // std::cout << "8" << std::endl;
          }
          // std::cout << "9" << std::endl;
          // consider sub metadatas
          for (const auto& [name, metadata_id]: metadata_object->sub_metadata_ids) {
            // std::cout << "10" << std::endl;
            auto sub_metadata = std::make_unique<draco::Metadata>();
            // std::cout << "11 " << sub_metadata.get() << ' ' << &metadatas[metadata_id] << std::endl;
            to_parse_metadatas_next.push_back({sub_metadata.get(), &metadatas[metadata_id]});
            // std::cout << "12" << std::endl;
            metadata->AddSubMetadata(name, std::move(sub_metadata));
            // std::cout << "13" << std::endl;
          }
        }
        // std::cout << "finish cycle iteration" << std::endl;
        // std::cout << "to_parse_metadata_next before" << std::endl;
        // for (auto& pair: to_parse_metadatas_next)
        //   std::cout << pair.first << ' ' << pair.second << std::endl;
        // std::cout << "to_parse_metadata before" << std::endl;
        // for (auto& pair: to_parse_metadata)
        //     std::cout << pair.first << ' ' << pair.second << std::endl;
        to_parse_metadata = std::move(to_parse_metadatas_next);
        // std::cout << "to_parse_metadata_next after" << std::endl;
        //   for (auto& pair: to_parse_metadatas_next)
        //     std::cout << pair.first << ' ' << pair.second << std::endl;
        // std::cout << "to_parse_metadata after" << std::endl;
        // for (auto& pair: to_parse_metadata)
        //     std::cout << pair.first << ' ' << pair.second << std::endl;
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
      std::cout << "encode_metadata geometry_metadata" << std::endl;
      encode_metadata(metadatas, metadatas[gmo.metadata_id], geometry_metadata);

      for (const auto& attribute_object: gmo.generic_attributes) {
        std::cout << "encode_generic_attribute" << std::endl;
        int32_t attribute_id = encode_generic_attribute(attribute_object, pc);
        if (attribute_id < 0)
          continue;
        auto attribute_metadata = std::make_unique<draco::AttributeMetadata>();
        std::cout << "encode_metadata" << std::endl;
        encode_metadata(metadatas, metadatas[attribute_object.metadata_id], *attribute_metadata);
        std::cout << "AddAttributeMetadata " << attribute_id << std::endl;
        

        auto& attribute_metadata_ref = *attribute_metadata;
        std::string attr_name;
        attribute_metadata->GetEntryString("name", &attr_name);
        std::cout << "put attribute metadata with name " << attr_name << std::endl;
        pc.AddAttributeMetadata(attribute_id, std::move(attribute_metadata));
        std::cout << attribute_metadata_ref.att_unique_id();
      }
      std::cout << "encode_geometry_metadata finish" << std::endl;
      
      return true;
    }
  }


  MeshObject decode_buffer(const char *buffer, std::size_t buffer_len) {
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
    std::cout << "decode_buffer" << std::endl;
    std::cout << "mesh points " << mesh->num_points() << std::endl;
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
      std::cout << '[' << v.value() << "] ";
      std::cout << pos_val[0] << ' ' << pos_val[1] << ' ' << pos_val[2] << std::endl;
      meshObject.points.push_back(pos_val[0]);
      meshObject.points.push_back(pos_val[1]);
      meshObject.points.push_back(pos_val[2]);
    }
    for (draco::FaceIndex i(0); i < mesh->num_faces(); ++i) {
      std::cout << '[' << i.value() << "] ";
      const auto &f = mesh->face(i);
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[0]))));
      std::cout << meshObject.faces.back() << ' ';
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[1]))));
      std::cout << meshObject.faces.back() << ' ';
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[2]))));
      std::cout << meshObject.faces.back() << ' ';
      std::cout << std::endl;
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
      std::cout << "result metadatas BEGIN" << std::endl;
      for (auto& m : meshObject.metadatas)
        std::cout << &m << std::endl;
      std::cout << "result metadatas END" << std::endl;
    }
    meshObject.decode_status = successful;
    std::cout << " meshObject " << meshObject.points.size() << std::endl;
    return meshObject;
  }

  PointCloudObject decode_buffer_to_point_cloud(const char *buffer, std::size_t buffer_len) {
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
    std::cout << "encode_mesh" << std::endl;
    std::cout << " meshObject points " << points.size() << std::endl;
    for (std::size_t i = 0; i <= faces.size() - 3; i += 3) {
      auto point1Index = faces[i]*3;
      auto point2Index = faces[i+1]*3;
      auto point3Index = faces[i+2]*3;
      std::cout << '[' << i << "] " << point1Index << ' ' << point2Index << ' ' << point3Index << ' ';
      std::cout << points[point1Index] << ' ' << points[point2Index] << ' ';
      std::cout << points[point3Index] << std::endl;
      mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i), 
                                   draco::Vector3f(points[point1Index], points[point1Index+1], points[point1Index+2]).data(),
                                   draco::Vector3f(points[point2Index], points[point2Index+1], points[point2Index+2]).data(),
                                   draco::Vector3f(points[point3Index], points[point3Index+1], points[point3Index+2]).data());
    }

    std::unique_ptr<draco::Mesh> ptr_mesh = mb.Finalize();
    draco::Mesh *mesh = ptr_mesh.get();
    std::cout << " mesh points " << mesh->num_points() << std::endl;
    
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
