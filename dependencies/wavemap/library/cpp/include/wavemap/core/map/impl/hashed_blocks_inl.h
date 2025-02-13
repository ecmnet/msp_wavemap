#ifndef WAVEMAP_CORE_MAP_IMPL_HASHED_BLOCKS_INL_H_
#define WAVEMAP_CORE_MAP_IMPL_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>
#include <unordered_set>

#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"

namespace wavemap {
inline FloatingPoint HashedBlocks::getCellValue(const Index3D& index) const {
  return getValueOrDefault(index);
}

inline void HashedBlocks::setCellValue(const Index3D& index,
                                       FloatingPoint new_value) {
  getOrAllocateValue(index) = new_value;
}

inline void HashedBlocks::addToCellValue(const Index3D& index,
                                         FloatingPoint update) {
  FloatingPoint& cell_data = getOrAllocateValue(index);
  cell_data = clampedAdd(cell_data, update);
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_IMPL_HASHED_BLOCKS_INL_H_
