// SPDX-License-Identifier: MIT
// Uses the installed Nav2 map loader offline, never creates a ROS Node.
#include "cmr_mission/delivery.hpp"
#include <nav2_map_server/map_io.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <filesystem>
#include <cmath>

int main(int argc, char **argv) {
  using namespace cmr::mission;
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      std::cout << "map_identity --map absolute.yaml --output new.json (offline; no Node)\n";
      return 0;
    }
    if (argc != 5 || std::string(argv[1]) != "--map" || std::string(argv[3]) != "--output"
        || !std::filesystem::path(argv[2]).is_absolute() || std::filesystem::exists(argv[4])) {
      throw std::runtime_error("invalid paths or output exists");
    }
    nav_msgs::msg::OccupancyGrid map;
    if (nav2_map_server::loadMapFromYaml(argv[2], map) != nav2_map_server::LOAD_MAP_SUCCESS) {
      throw std::runtime_error("installed map loader rejected map");
    }
    auto q = map.info.origin.orientation;
    Json data = {
        {"width", map.info.width},
        {"height", map.info.height},
        {"resolution", map.info.resolution},
        {"origin_x", map.info.origin.position.x},
        {"origin_y", map.info.origin.position.y},
        {"origin_yaw", std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)},
        {"grid_sha256", bytes_sha256(std::string(reinterpret_cast<const char *>(map.data.data()),
                                                 map.data.size()))}};
    const auto text = data.dump(2) + "\n";

    struct Fd {
      int n;

      ~Fd() {
        if (n >= 0) {
          close(n);
        }
      }
    } fd{open(argv[4], O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600)};

    if (fd.n < 0) {
      throw std::runtime_error("exclusive output open");
    }
    std::size_t offset = 0;
    while (offset < text.size()) {
      auto n = write(fd.n, text.data() + offset, text.size() - offset);
      if (n <= 0) {
        throw std::runtime_error("output write");
      }
      offset += static_cast<std::size_t>(n);
    }
    if (fsync(fd.n)) {
      throw std::runtime_error("output sync");
    }
    std::cout << "MAP_IDENTITY_OK cells=" << map.data.size() << "\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "MAP_IDENTITY_ERROR " << e.what() << '\n';
    return 2;
  }
}
