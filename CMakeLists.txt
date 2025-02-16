cmake_minimum_required(VERSION 3.5)
project(msp_wavemap)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-address-of-packed-member -O3 -march=native )
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/wavemap/library/cpp wavemap)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/waverider/waverider)

include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/wavemap/library/cpp/include
                            ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/waverider/waverider/include)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(msp_msgs REQUIRED)
find_package(msp_controller REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(${PROJECT_NAME}  src/msp_wavemap.cpp 
                                src/msp_waverider.cpp
                                lib/config/yaml_cpp_conversions.cc
                                lib/config/stream_conversions.cc
                                lib/config/file_conversions.cc
                               # lib/utils/tf_transformer.cc
                                lib/map_operations/crop_map_operation.cc
                                lib/map_operations/msp_map_operation_factory.cc
                                
                              )
include_directories(include dependencies/wavemap/library/cpp/include
                            dependencies/waverider/waverider/include
                            dependencies/rmpcpp/include
                            )
                                          
ament_target_dependencies(${PROJECT_NAME} msp_controller 
                   msp_msgs px4_msgs rclcpp 
                   tf2 tf2_ros 
                   gz-transport13 
                   Eigen3 
                   pcl_conversions 
                   cv_bridge 
                   OpenCV 
                   geometry_msgs
                    )

ament_export_include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/wavemap/library/cpp/include
                                         ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/waverider/waverider/include)

target_link_libraries(${PROJECT_NAME} 
                      yaml-cpp 
                      Eigen3::Eigen
                      waverider
                      wavemap::wavemap_core
                      wavemap::wavemap_io
                      wavemap::wavemap_pipeline
)

#target_include_directories(${PROJECT_NAME})


ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime )


# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME}
)

ament_package()
