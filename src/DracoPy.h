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

namespace {
  void encode(std::ostream& o_stream, const uint32_t& value)
  {
    o_stream.write(reinterpret_cast<const char*>(&value), 4);
  }

   void encode(std::ostream& o_stream, const uint8_t* data, uint32_t data_len)
  {
    encode(o_stream, data_len);
    if (data_len > 0)
        o_stream.write(reinterpret_cast<const char*>(data), data_len);
  }

  template<class Item>
  void encode(std::ostream& o_stream, const std::vector<Item>& v)
  {
    encode(o_stream, reinterpret_cast<const uint8_t*>(v.data()),
           static_cast<uint32_t>(v.size() * sizeof(Item)));
  }

  void encode(std::ostream& o_stream, const std::string& s)
  {
    encode(o_stream, reinterpret_cast<const uint8_t*>(s.data()),
           static_cast<uint32_t>(s.size()));
  }
}

namespace DracoFunctions {
    using namespace draco;

  enum decoding_status { successful, not_draco_encoded, no_position_attribute, failed_during_decoding };
  enum encoding_status { successful_encoding, failed_during_encoding };

  enum endian_order : uint32_t {
    little_endian   = 0x00000001,
    big_endian      = 0x01000000,
    unknown_endian  = 0xFFFFFFFF
  };

  struct PointCloudObject {
    std::vector<float> points;

    // Encoding options stored in metadata
    bool encoding_options_set;
    int quantization_bits;
    double quantization_range;
    std::vector<double> quantization_origin;

    decoding_status decode_status;

    // not parsed metadata
    std::string binary_metadata;
  };

  struct MeshObject : PointCloudObject {
    std::vector<float> normals;
    std::vector<unsigned int> faces;
  };

  struct EncodedObject {
    std::vector<unsigned char> buffer;
    encoding_status encode_status;
  };

    /**
     * A simple function that can be used to help determine a program's endianness
     * at compile-time.
     */
    constexpr endian_order get_endian_order() {
        return
            ((0xFFFFFFFF & 1) == little_endian)
                ? little_endian
                : ((0xFFFFFFFF & 1) == big_endian)
                    ? big_endian
                    : unknown_endian;
    }

  std::string encodeDracoMetadata(const GeometryMetadata& geometry_metadata) {
    std::stringstream ss;
    std::vector<const Metadata*> to_parse_metadata =
            { {static_cast<const Metadata*>(&geometry_metadata)} };
    // consider attribute metadatas
    const auto& attribute_metadatas = geometry_metadata.attribute_metadatas();
    encode(ss, static_cast<uint32_t>(attribute_metadatas.size()));
    for (const auto& attribute_metadata: attribute_metadatas)
        to_parse_metadata.push_back(
                static_cast<Metadata*>(attribute_metadata.get()));
    // encode metadatas level by level
    for (std::vector<const Metadata*> to_parse_metadatas_next;
            !to_parse_metadata.empty();
            to_parse_metadata = std::move(to_parse_metadatas_next)) {
      for (const auto* draco_metadata: to_parse_metadata) {
        // encode entries
        const auto& entries = draco_metadata->entries();
        encode(ss, static_cast<uint32_t>(entries.size()));
        for (const auto& [name, value]: draco_metadata->entries()) {
            encode(ss, name);
            encode(ss, value.data());
        }
        // consider sub metadatas
        const auto& sub_metadatas = draco_metadata->sub_metadatas();
        encode(ss, static_cast<uint32_t>(sub_metadatas.size()));
        for (const auto& [name, draco_sub_metadata]: sub_metadatas) {
            encode(ss, name);
            to_parse_metadatas_next.push_back(draco_sub_metadata.get());
        }
      }
    }
    return ss.str();
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
      meshObject.binary_metadata = encodeDracoMetadata(*metadata);
    }
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
      pointCloudObject.binary_metadata = encodeDracoMetadata(*metadata);
    }
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

  EncodedObject encode_mesh(const std::vector<float> &points, const std::vector<unsigned int> &faces,
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
