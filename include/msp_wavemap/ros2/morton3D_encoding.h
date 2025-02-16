#pragma once
#include <iostream>
#include <cstdint>
#include <cmath>

constexpr float MAX_INT2 = 2097151.0f/2.0f; // 21-bit max integer value

// Function to interleave 21-bit integers into Morton format
uint64_t splitBits3D(uint32_t x) {
    x &= 0x1FFFFF; // Ensure 21-bit input
    x = (x | (x << 32)) & 0x1F00000000FFFF;
    x = (x | (x << 16)) & 0x1F0000FF0000FF;
    x = (x | (x <<  8)) & 0x100F00F00F00F00F;
    x = (x | (x <<  4)) & 0x10C30C30C30C30C3;
    x = (x | (x <<  2)) & 0x1249249249249249; // Final bit interleave
    return x;
}

// Morton encoding for 3D float32 in range [0,1]
uint64_t mortonEncode3D(float x, float y, float z) {
    uint32_t xi = static_cast<uint32_t>((x + MAX_INT2) * MAX_INT2);
    uint32_t yi = static_cast<uint32_t>((y + MAX_INT2) * MAX_INT2);
    uint32_t zi = static_cast<uint32_t>((z * MAX_INT2) * MAX_INT2);
    
    return (splitBits3D(xi) | (splitBits3D(yi) << 1) | (splitBits3D(zi) << 2));
}
