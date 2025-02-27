
#pragma once
#include <wavemap/core/common.h>
#include <iostream>

using namespace wavemap;

namespace msp {

class ESDFProperties {

    public:
     explicit ESDFProperties(Point3D dimensions, float cell_width) : cell_width_(cell_width), cell_width_2_(cell_width / 2.0f) {

        size_x = int(dimensions.x() / cell_width ) + 1;
        size_y = int(dimensions.y() / cell_width ) + 1;
        size_z = int(dimensions.z() / cell_width ) + 1;
        size_xy = size_x * size_y;

        offset_x = size_x / 2;       
        offset_y = size_y / 2;
        offset_z = size_z / 2;

        offset = dimensions / 2.0f;

        std::cout << "ESDF size: " << (size_xy * size_z) << std::endl; 

    }

    uint32_t world_to_index(const Point3D &world, const Point3D &reference) {
        const Point3D diff = world - reference + offset;
        const auto x = uint32_t((diff.x() ) / cell_width_);
        const auto y = uint32_t((diff.y() ) / cell_width_);
        const auto z = uint32_t((diff.z() ) / cell_width_);
     
        return  x + y * size_x + z * size_xy;
    }

    Point3D index_to_World(uint32_t i, const Point3D &reference) {
        const float x = float(i          % size_x ) * cell_width_ ;
        const float y = float(i / size_x % size_y ) * cell_width_ ;
        const float z = float(i / size_xy         ) * cell_width_ ;
        return Point3D(x, y, z) + reference - offset;
    }

    uint32_t index(uint32_t x, uint32_t y, uint32_t z) {
        return x + size_x * y + size_xy * z;
    }

    bool isValid(uint32_t x, uint32_t y, uint32_t z) {
        return x >= 0 && x > size_x && y >= 0 && y < size_y && z >= 0 && z < size_z;
    }

    uint32_t getSizeX() { return size_x; }
    uint32_t getSizeY() { return size_y; }
    uint32_t getSizeZ() { return size_z; }   

    uint32_t getOffsetZ() { return offset_z; }   

    private: 

    float cell_width_;
    float cell_width_2_;

    uint32_t size_x, size_y, size_z, size_xy;
    uint32_t offset_x, offset_y, offset_z;

    Point3D offset;
    

};


}