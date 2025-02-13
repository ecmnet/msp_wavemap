cmake_minimum_required(VERSION 3.0.2)


find_package(Eigen3 REQUIRED)

# Declare the wavemap core library
add_library(waverider)

include_directories(include ${PROJECT_SOURCE_DIR}/dependencies/wavemap/library/cpp/include
                            ${PROJECT_SOURCE_DIR}/dependencies/rmpcpp/include)

# Configure target properties, include directories and linking
set_wavemap_target_properties(waverider)
add_wavemap_include_directories(waverider)
target_link_libraries(waverider
    PUBLIC Eigen3::Eigen glog Boost::preprocessor)

# Libraries
target_sources(waverider
    PRIVATE
    src/goal_policy.cc
    src/goal_policy_tuning.cc
    src/obstacle_filter.cc
    src/obstacle_list_policy_.cc
    src/parallelized_policy.cc
    src/obstacle_policy_tuning.cc
    src/waverider_policy.cc
    src/yaw_policy.cc
    src/yaw_policy_tuning.cc)
    

# Export
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/waverider/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/waverider
      FILES_MATCHING PATTERN "*.h")

  # Mark target for installation and add it to the CMake exports file
  install(TARGETS waverider
      EXPORT waverider-targets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

