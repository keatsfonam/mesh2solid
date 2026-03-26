std::string lowercase_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string local_xml_name(std::string name) {
  const std::size_t separator = name.find(':');
  if (separator != std::string::npos) {
    name = name.substr(separator + 1);
  }
  return lowercase_copy(std::move(name));
}

class SimpleXmlParser {
 public:
  explicit SimpleXmlParser(std::string text) : text_(std::move(text)) {}

  XmlNode parse() {
    XmlNode document;
    document.name = "#document";
    skip_misc();
    while (position_ < text_.size()) {
      if (text_[position_] != '<') {
        ++position_;
        continue;
      }
      document.children.push_back(parse_element());
      skip_misc();
    }
    return document;
  }

 private:
  void skip_whitespace() {
    while (position_ < text_.size() &&
           std::isspace(static_cast<unsigned char>(text_[position_]))) {
      ++position_;
    }
  }

  bool starts_with(const std::string& token) const {
    return text_.compare(position_, token.size(), token) == 0;
  }

  void skip_until(const std::string& token) {
    const std::size_t found = text_.find(token, position_);
    if (found == std::string::npos) {
      throw std::runtime_error("Malformed XML: expected token " + token);
    }
    position_ = found + token.size();
  }

  void skip_misc() {
    while (position_ < text_.size()) {
      skip_whitespace();
      if (starts_with("<?")) {
        skip_until("?>");
        continue;
      }
      if (starts_with("<!--")) {
        skip_until("-->");
        continue;
      }
      if (starts_with("<![CDATA[")) {
        skip_until("]]>");
        continue;
      }
      break;
    }
  }

  std::string parse_name() {
    const std::size_t start = position_;
    while (position_ < text_.size()) {
      const char character = text_[position_];
      if (std::isalnum(static_cast<unsigned char>(character)) || character == '_' ||
          character == ':' || character == '-' || character == '.') {
        ++position_;
      } else {
        break;
      }
    }
    if (start == position_) {
      throw std::runtime_error("Malformed XML: expected tag or attribute name");
    }
    return text_.substr(start, position_ - start);
  }

  std::string parse_attribute_value() {
    skip_whitespace();
    if (position_ >= text_.size() || (text_[position_] != '"' && text_[position_] != '\'')) {
      throw std::runtime_error("Malformed XML: expected quoted attribute value");
    }
    const char quote = text_[position_++];
    std::string value;
    while (position_ < text_.size() && text_[position_] != quote) {
      value.push_back(text_[position_++]);
    }
    if (position_ >= text_.size()) {
      throw std::runtime_error("Malformed XML: unterminated attribute value");
    }
    ++position_;
    return value;
  }

  XmlNode parse_element() {
    if (position_ >= text_.size() || text_[position_] != '<') {
      throw std::runtime_error("Malformed XML: expected start tag");
    }
    ++position_;
    if (position_ < text_.size() && text_[position_] == '/') {
      throw std::runtime_error("Malformed XML: unexpected closing tag");
    }

    XmlNode node;
    node.name = local_xml_name(parse_name());

    while (true) {
      skip_whitespace();
      if (position_ >= text_.size()) {
        throw std::runtime_error("Malformed XML: unexpected end of input");
      }
      if (starts_with("/>")) {
        position_ += 2;
        return node;
      }
      if (text_[position_] == '>') {
        ++position_;
        break;
      }

      std::string attribute_name = local_xml_name(parse_name());
      skip_whitespace();
      if (position_ >= text_.size() || text_[position_] != '=') {
        throw std::runtime_error("Malformed XML: expected '=' after attribute");
      }
      ++position_;
      node.attributes[attribute_name] = parse_attribute_value();
    }

    while (true) {
      skip_misc();
      if (position_ >= text_.size()) {
        throw std::runtime_error("Malformed XML: missing closing tag for " + node.name);
      }
      if (starts_with("</")) {
        position_ += 2;
        const std::string closing_name = local_xml_name(parse_name());
        if (closing_name != node.name) {
          throw std::runtime_error("Malformed XML: mismatched closing tag");
        }
        skip_whitespace();
        if (position_ >= text_.size() || text_[position_] != '>') {
          throw std::runtime_error("Malformed XML: malformed closing tag");
        }
        ++position_;
        return node;
      }
      if (text_[position_] == '<') {
        node.children.push_back(parse_element());
      } else {
        ++position_;
      }
    }
  }

  std::string text_;
  std::size_t position_ {};
};

const XmlNode* find_first_child(const XmlNode& parent, const std::string& name) {
  const std::string lowered_name = lowercase_copy(name);
  for (const XmlNode& child : parent.children) {
    if (child.name == lowered_name) {
      return &child;
    }
  }
  return nullptr;
}

std::vector<const XmlNode*> find_children(const XmlNode& parent, const std::string& name) {
  const std::string lowered_name = lowercase_copy(name);
  std::vector<const XmlNode*> matches;
  for (const XmlNode& child : parent.children) {
    if (child.name == lowered_name) {
      matches.push_back(&child);
    }
  }
  return matches;
}

std::string attribute_or_empty(const XmlNode& node, const std::string& name) {
  const auto found = node.attributes.find(lowercase_copy(name));
  return found == node.attributes.end() ? std::string() : found->second;
}

std::vector<std::uint8_t> read_binary_file(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }
  input.seekg(0, std::ios::end);
  const std::streamoff size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (size < 0) {
    throw std::runtime_error("Unable to determine file size: " + path.string());
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!input && !bytes.empty()) {
    throw std::runtime_error("Unable to read input file: " + path.string());
  }
  return bytes;
}

std::uint16_t read_le16(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return static_cast<std::uint16_t>(bytes[offset]) |
         (static_cast<std::uint16_t>(bytes[offset + 1]) << 8U);
}

std::uint32_t read_le32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return static_cast<std::uint32_t>(bytes[offset]) |
         (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

std::string normalize_zip_path(std::string path) {
  std::replace(path.begin(), path.end(), '\\', '/');
  while (!path.empty() && (path.front() == '/' || path.front() == '.')) {
    if (path.front() == '.') {
      if (path.size() >= 2 && path[1] == '/') {
        path.erase(0, 2);
      } else {
        path.erase(path.begin());
      }
    } else {
      path.erase(path.begin());
    }
  }

  std::filesystem::path normalized = std::filesystem::path(path).lexically_normal();
  path = normalized.generic_string();
  while (!path.empty() && path.front() == '/') {
    path.erase(path.begin());
  }
  return path;
}

ZipArchive read_zip_archive(const std::filesystem::path& path) {
  ZipArchive archive;
  archive.bytes = read_binary_file(path);
  if (archive.bytes.size() < 22) {
    throw std::runtime_error("3MF package is too small to be a valid ZIP archive: " + path.string());
  }

  const std::uint32_t eocd_signature = 0x06054b50U;
  std::size_t eocd_offset = std::string::npos;
  const std::size_t search_start =
      archive.bytes.size() > 65557 ? archive.bytes.size() - 65557 : 0;
  for (std::size_t offset = archive.bytes.size() - 22; offset + 1 > search_start; --offset) {
    if (read_le32(archive.bytes, offset) == eocd_signature) {
      eocd_offset = offset;
      break;
    }
    if (offset == 0) {
      break;
    }
  }
  if (eocd_offset == std::string::npos) {
    throw std::runtime_error("Unable to locate ZIP central directory in 3MF package: " +
                             path.string());
  }

  const std::uint16_t entry_count = read_le16(archive.bytes, eocd_offset + 10);
  const std::uint32_t central_directory_size = read_le32(archive.bytes, eocd_offset + 12);
  const std::uint32_t central_directory_offset = read_le32(archive.bytes, eocd_offset + 16);

  if (central_directory_offset + central_directory_size > archive.bytes.size()) {
    throw std::runtime_error("Malformed ZIP central directory in 3MF package: " + path.string());
  }

  std::size_t cursor = central_directory_offset;
  for (std::uint16_t index = 0; index < entry_count; ++index) {
    if (cursor + 46 > archive.bytes.size() || read_le32(archive.bytes, cursor) != 0x02014b50U) {
      throw std::runtime_error("Malformed ZIP directory entry in 3MF package: " + path.string());
    }

    const std::uint16_t compression_method = read_le16(archive.bytes, cursor + 10);
    const std::uint32_t compressed_size = read_le32(archive.bytes, cursor + 20);
    const std::uint32_t uncompressed_size = read_le32(archive.bytes, cursor + 24);
    const std::uint16_t file_name_length = read_le16(archive.bytes, cursor + 28);
    const std::uint16_t extra_length = read_le16(archive.bytes, cursor + 30);
    const std::uint16_t comment_length = read_le16(archive.bytes, cursor + 32);
    const std::uint32_t local_header_offset = read_le32(archive.bytes, cursor + 42);

    if (cursor + 46 + file_name_length + extra_length + comment_length > archive.bytes.size()) {
      throw std::runtime_error("Malformed ZIP filename in 3MF package: " + path.string());
    }

    std::string name(reinterpret_cast<const char*>(archive.bytes.data() + cursor + 46),
                     file_name_length);
    name = normalize_zip_path(std::move(name));
    archive.entries[name] = {name, compression_method, compressed_size, uncompressed_size,
                             local_header_offset};

    cursor += 46 + file_name_length + extra_length + comment_length;
  }

  return archive;
}

std::vector<std::uint8_t> unzip_entry(const ZipArchive& archive, const ZipEntry& entry) {
  if (entry.local_header_offset + 30 > archive.bytes.size() ||
      read_le32(archive.bytes, entry.local_header_offset) != 0x04034b50U) {
    throw std::runtime_error("Malformed ZIP local file header in 3MF package");
  }

  const std::uint16_t file_name_length = read_le16(archive.bytes, entry.local_header_offset + 26);
  const std::uint16_t extra_length = read_le16(archive.bytes, entry.local_header_offset + 28);
  const std::size_t data_offset =
      entry.local_header_offset + 30 + file_name_length + extra_length;
  if (data_offset + entry.compressed_size > archive.bytes.size()) {
    throw std::runtime_error("Malformed ZIP file data in 3MF package");
  }

  const std::uint8_t* compressed = archive.bytes.data() + data_offset;
  if (entry.compression_method == 0) {
    return std::vector<std::uint8_t>(compressed, compressed + entry.compressed_size);
  }
  if (entry.compression_method != 8) {
    throw std::runtime_error("Unsupported ZIP compression method in 3MF package");
  }

  std::vector<std::uint8_t> uncompressed(entry.uncompressed_size);
  z_stream stream {};
  stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(compressed));
  stream.avail_in = entry.compressed_size;
  stream.next_out = reinterpret_cast<Bytef*>(uncompressed.data());
  stream.avail_out = entry.uncompressed_size;

  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
    throw std::runtime_error("Unable to initialize ZIP decompression for 3MF package");
  }
  const int status = inflate(&stream, Z_FINISH);
  inflateEnd(&stream);
  if (status != Z_STREAM_END) {
    throw std::runtime_error("Unable to decompress ZIP entry from 3MF package");
  }

  uncompressed.resize(stream.total_out);
  return uncompressed;
}

std::string unzip_text_entry(const ZipArchive& archive, const std::string& name) {
  const auto found = archive.entries.find(normalize_zip_path(name));
  if (found == archive.entries.end()) {
    throw std::runtime_error("Missing entry in 3MF package: " + name);
  }
  const std::vector<std::uint8_t> bytes = unzip_entry(archive, found->second);
  return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::string resolve_zip_target(const std::string& source_part, const std::string& target) {
  if (!target.empty() && target.front() == '/') {
    return normalize_zip_path(target.substr(1));
  }

  const std::filesystem::path source_path = std::filesystem::path(source_part).parent_path();
  return normalize_zip_path((source_path / target).generic_string());
}

std::string find_3mf_model_part(const ZipArchive& archive) {
  auto direct = archive.entries.find("3D/3dmodel.model");
  if (direct != archive.entries.end()) {
    return direct->first;
  }

  if (archive.entries.count("_rels/.rels") > 0) {
    const XmlNode relationships_document = SimpleXmlParser(unzip_text_entry(archive, "_rels/.rels")).parse();
    const XmlNode* relationships = find_first_child(relationships_document, "relationships");
    if (relationships != nullptr) {
      for (const XmlNode* relationship : find_children(*relationships, "relationship")) {
        const std::string type = attribute_or_empty(*relationship, "type");
        if (type.find("3dmodel") != std::string::npos) {
          const std::string target = attribute_or_empty(*relationship, "target");
          if (!target.empty()) {
            return resolve_zip_target("", target);
          }
        }
      }
    }
  }

  for (const auto& [name, entry] : archive.entries) {
    (void)entry;
    if (name.size() >= 6 && lowercase_copy(name.substr(name.size() - 6)) == ".model") {
      return name;
    }
  }

  throw std::runtime_error("Unable to find model part in 3MF package");
}

double parse_required_double(const std::string& value, const std::string& context) {
  try {
    return std::stod(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid floating-point value in " + context + ": " + value);
  }
}

int parse_required_int(const std::string& value, const std::string& context) {
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid integer value in " + context + ": " + value);
  }
}

AffineTransform identity_transform() {
  return {};
}

AffineTransform combine_transforms(const AffineTransform& lhs, const AffineTransform& rhs) {
  AffineTransform combined;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      combined.values[row][column] = 0.0;
      for (int inner = 0; inner < 4; ++inner) {
        combined.values[row][column] += lhs.values[row][inner] * rhs.values[inner][column];
      }
    }
  }
  return combined;
}

Vec3 apply_transform(const AffineTransform& transform, const Vec3& point) {
  return {
      point.x * transform.values[0][0] + point.y * transform.values[1][0] +
          point.z * transform.values[2][0] + transform.values[3][0],
      point.x * transform.values[0][1] + point.y * transform.values[1][1] +
          point.z * transform.values[2][1] + transform.values[3][1],
      point.x * transform.values[0][2] + point.y * transform.values[1][2] +
          point.z * transform.values[2][2] + transform.values[3][2],
  };
}

AffineTransform parse_3mf_transform(const std::string& value) {
  if (value.empty()) {
    return identity_transform();
  }

  std::istringstream parser(value);
  std::array<double, 12> terms {};
  for (double& term : terms) {
    if (!(parser >> term)) {
      throw std::runtime_error("Malformed 3MF transform attribute");
    }
  }

  AffineTransform transform;
  transform.values = {{
      {terms[0], terms[1], terms[2], 0.0},
      {terms[3], terms[4], terms[5], 0.0},
      {terms[6], terms[7], terms[8], 0.0},
      {terms[9], terms[10], terms[11], 1.0},
  }};
  return transform;
}

double unit_scale_to_millimeters(const std::string& unit_name) {
  const std::string unit = lowercase_copy(unit_name);
  if (unit.empty() || unit == "millimeter") {
    return 1.0;
  }
  if (unit == "micron") {
    return 0.001;
  }
  if (unit == "centimeter") {
    return 10.0;
  }
  if (unit == "meter") {
    return 1000.0;
  }
  if (unit == "inch") {
    return 25.4;
  }
  if (unit == "foot") {
    return 304.8;
  }
  throw std::runtime_error("Unsupported 3MF unit: " + unit_name);
}

bool file_looks_binary_stl(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }

  input.seekg(0, std::ios::end);
  const std::streamoff size = input.tellg();
  if (size < 84) {
    return false;
  }

  input.seekg(80, std::ios::beg);
  std::uint32_t triangle_count = 0;
  input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
  if (!input) {
    return false;
  }

  const std::uint64_t expected_size = 84ULL + static_cast<std::uint64_t>(triangle_count) * 50ULL;
  return expected_size == static_cast<std::uint64_t>(size);
}

MeshModel load_ascii_stl(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }

  MeshModel mesh;
  mesh.source_format = "ascii_stl";

  std::vector<int> current_triangle_vertices;
  current_triangle_vertices.reserve(3);
  std::string line;

  while (std::getline(input, line)) {
    std::istringstream parser(line);
    std::string keyword;
    parser >> keyword;
    if (keyword != "vertex") {
      continue;
    }

    Vec3 vertex {};
    parser >> vertex.x >> vertex.y >> vertex.z;
    if (!parser) {
      throw std::runtime_error("Malformed vertex in ASCII STL: " + path.string());
    }

    mesh.vertices.push_back(vertex);
    current_triangle_vertices.push_back(static_cast<int>(mesh.vertices.size() - 1));
    if (current_triangle_vertices.size() == 3) {
      Triangle triangle;
      triangle.vertices = {current_triangle_vertices[0], current_triangle_vertices[1],
                           current_triangle_vertices[2]};
      mesh.triangles.push_back(triangle);
      current_triangle_vertices.clear();
    }
  }

  if (!current_triangle_vertices.empty()) {
    throw std::runtime_error("ASCII STL ended mid-triangle: " + path.string());
  }

  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles found in ASCII STL: " + path.string());
  }

  return mesh;
}

MeshModel load_binary_stl(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }

  MeshModel mesh;
  mesh.source_format = "binary_stl";

  char header[80] {};
  input.read(header, sizeof(header));
  std::uint32_t triangle_count = 0;
  input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
  if (!input) {
    throw std::runtime_error("Malformed binary STL header: " + path.string());
  }

  mesh.vertices.reserve(static_cast<std::size_t>(triangle_count) * 3);
  mesh.triangles.reserve(triangle_count);

  for (std::uint32_t i = 0; i < triangle_count; ++i) {
    float facet_normal[3] {};
    float facet_vertices[9] {};
    std::uint16_t attribute_byte_count = 0;
    input.read(reinterpret_cast<char*>(facet_normal), sizeof(facet_normal));
    input.read(reinterpret_cast<char*>(facet_vertices), sizeof(facet_vertices));
    input.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(attribute_byte_count));
    if (!input) {
      throw std::runtime_error("Malformed binary STL facet: " + path.string());
    }

    Triangle triangle;
    for (int vertex_index = 0; vertex_index < 3; ++vertex_index) {
      const Vec3 vertex {facet_vertices[vertex_index * 3], facet_vertices[vertex_index * 3 + 1],
                         facet_vertices[vertex_index * 3 + 2]};
      mesh.vertices.push_back(vertex);
      triangle.vertices[vertex_index] = static_cast<int>(mesh.vertices.size() - 1);
    }
    mesh.triangles.push_back(triangle);
  }

  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles found in binary STL: " + path.string());
  }

  return mesh;
}

void recompute_mesh_geometry(MeshModel& mesh) {
  mesh.bounds = {};
  for (const Vec3& vertex : mesh.vertices) {
    include_point(mesh.bounds, vertex);
  }

  for (Triangle& triangle : mesh.triangles) {
    const Vec3& a = mesh.vertices[triangle.vertices[0]];
    const Vec3& b = mesh.vertices[triangle.vertices[1]];
    const Vec3& c = mesh.vertices[triangle.vertices[2]];
    triangle.area = triangle_area(a, b, c);
    triangle.normal = triangle.area > 1e-12 ? triangle_normal(a, b, c) : Vec3 {0.0, 0.0, 0.0};
  }
}

MeshModel load_stl(const std::filesystem::path& path) {
  MeshModel mesh = file_looks_binary_stl(path) ? load_binary_stl(path) : load_ascii_stl(path);
  recompute_mesh_geometry(mesh);
  return mesh;
}

MeshModel load_3mf(const std::filesystem::path& path) {
  const ZipArchive archive = read_zip_archive(path);
  const std::string model_part = find_3mf_model_part(archive);
  const XmlNode document = SimpleXmlParser(unzip_text_entry(archive, model_part)).parse();
  const XmlNode* model = find_first_child(document, "model");
  if (model == nullptr) {
    throw std::runtime_error("Malformed 3MF document: missing <model>");
  }

  const double unit_scale = unit_scale_to_millimeters(attribute_or_empty(*model, "unit"));
  const XmlNode* resources = find_first_child(*model, "resources");
  if (resources == nullptr) {
    throw std::runtime_error("Malformed 3MF document: missing <resources>");
  }

  std::unordered_map<int, ThreeMFObject> objects;
  for (const XmlNode* object_node : find_children(*resources, "object")) {
    const int object_id = parse_required_int(attribute_or_empty(*object_node, "id"), "3MF object id");
    ThreeMFObject object;
    object.id = object_id;

    if (const XmlNode* mesh_node = find_first_child(*object_node, "mesh")) {
      const XmlNode* vertices_node = find_first_child(*mesh_node, "vertices");
      const XmlNode* triangles_node = find_first_child(*mesh_node, "triangles");
      if (vertices_node == nullptr || triangles_node == nullptr) {
        throw std::runtime_error("Malformed 3MF mesh object: missing <vertices> or <triangles>");
      }

      for (const XmlNode* vertex_node : find_children(*vertices_node, "vertex")) {
        object.vertices.push_back({
            parse_required_double(attribute_or_empty(*vertex_node, "x"), "3MF vertex x") * unit_scale,
            parse_required_double(attribute_or_empty(*vertex_node, "y"), "3MF vertex y") * unit_scale,
            parse_required_double(attribute_or_empty(*vertex_node, "z"), "3MF vertex z") * unit_scale,
        });
      }

      for (const XmlNode* triangle_node : find_children(*triangles_node, "triangle")) {
        object.triangles.push_back({
            parse_required_int(attribute_or_empty(*triangle_node, "v1"), "3MF triangle v1"),
            parse_required_int(attribute_or_empty(*triangle_node, "v2"), "3MF triangle v2"),
            parse_required_int(attribute_or_empty(*triangle_node, "v3"), "3MF triangle v3"),
        });
      }
    }

    if (const XmlNode* components_node = find_first_child(*object_node, "components")) {
      for (const XmlNode* component_node : find_children(*components_node, "component")) {
        object.components.push_back({
            parse_required_int(attribute_or_empty(*component_node, "objectid"), "3MF component objectid"),
            parse_3mf_transform(attribute_or_empty(*component_node, "transform")),
        });
      }
    }

    objects[object.id] = std::move(object);
  }

  std::vector<std::pair<int, AffineTransform>> build_items;
  if (const XmlNode* build_node = find_first_child(*model, "build")) {
    for (const XmlNode* item_node : find_children(*build_node, "item")) {
      build_items.push_back({
          parse_required_int(attribute_or_empty(*item_node, "objectid"), "3MF build item objectid"),
          parse_3mf_transform(attribute_or_empty(*item_node, "transform")),
      });
    }
  }

  if (build_items.empty()) {
    for (const auto& [object_id, object] : objects) {
      if (!object.vertices.empty() && !object.triangles.empty()) {
        build_items.push_back({object_id, identity_transform()});
      }
    }
  }

  if (build_items.empty()) {
    throw std::runtime_error("No build items or mesh objects found in 3MF document");
  }

  MeshModel mesh;
  mesh.source_format = "3mf";

  std::vector<int> recursion_stack;
  std::function<void(int, const AffineTransform&)> emit_object =
      [&](int object_id, const AffineTransform& transform) {
        const auto object_it = objects.find(object_id);
        if (object_it == objects.end()) {
          throw std::runtime_error("3MF build references unknown object id: " + std::to_string(object_id));
        }

        if (std::find(recursion_stack.begin(), recursion_stack.end(), object_id) != recursion_stack.end()) {
          throw std::runtime_error("3MF object graph contains a component cycle");
        }

        const ThreeMFObject& object = object_it->second;
        recursion_stack.push_back(object_id);

        if (!object.vertices.empty() && !object.triangles.empty()) {
          const int base_index = static_cast<int>(mesh.vertices.size());
          for (const Vec3& vertex : object.vertices) {
            mesh.vertices.push_back(apply_transform(transform, vertex));
          }
          for (const auto& triangle : object.triangles) {
            if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
                triangle[0] >= static_cast<int>(object.vertices.size()) ||
                triangle[1] >= static_cast<int>(object.vertices.size()) ||
                triangle[2] >= static_cast<int>(object.vertices.size())) {
              throw std::runtime_error("3MF triangle references an invalid vertex index");
            }
            Triangle emitted;
            emitted.vertices = {base_index + triangle[0], base_index + triangle[1], base_index + triangle[2]};
            mesh.triangles.push_back(emitted);
          }
        }

        for (const ThreeMFComponent& component : object.components) {
          emit_object(component.object_id, combine_transforms(component.transform, transform));
        }

        recursion_stack.pop_back();
      };

  for (const auto& [object_id, transform] : build_items) {
    emit_object(object_id, transform);
  }

  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles found in 3MF document");
  }

  recompute_mesh_geometry(mesh);
  return mesh;
}

MeshModel load_mesh(const std::filesystem::path& path) {
  const std::string extension = lowercase_copy(path.extension().string());
  if (extension == ".3mf") {
    return load_3mf(path);
  }
  return load_stl(path);
}
