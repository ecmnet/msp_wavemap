#include "msp_wavemap/lib/config/stream_conversions.h"
#include <yaml-cpp/yaml.h>
#include "msp_wavemap/lib/config/yaml_cpp_conversions.h"


namespace wavemap::io {
std::optional<param::Value> yamlStreamToParams(
    [[maybe_unused]] std::istream& istream) {

  try {
    YAML::Node yaml = YAML::Load(istream);
    return convert::yamlToParams(yaml);
  } catch (YAML::ParserException&) {
    LOG(WARNING) << "Failed to parse bytestream using yaml-cpp.";
    return std::nullopt;
  }

  LOG(ERROR) << "No YAML parser is available. Install yaml-cpp or add an "
                "interface to your preferred parser in wavemap/io/config.";
  return std::nullopt;
}
}  // namespace wavemap::io