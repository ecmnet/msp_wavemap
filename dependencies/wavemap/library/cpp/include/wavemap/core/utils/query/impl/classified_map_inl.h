#ifndef WAVEMAP_CORE_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
#define WAVEMAP_CORE_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_

#include <limits>
#include <utility>

namespace wavemap {
inline void ChildBitset::set(NdtreeIndexRelativeChild child_idx, bool value) {
  bitset = bit_ops::set_bit(bitset, child_idx, value);
}

inline bool ChildBitset::operator[](NdtreeIndexRelativeChild child_idx) const {
  return bit_ops::is_bit_set(bitset, child_idx);
}

inline bool ChildBitset::any() const {
  return bitset != static_cast<uint8_t>(0);
}

inline bool ClassifiedMap::has(const Index3D& index,
                               Occupancy::Id occupancy_type) const {
  return has(OctreeIndex{0, index}, occupancy_type);
}

inline bool ClassifiedMap::has(const OctreeIndex& index,
                               Occupancy::Id occupancy_type) const {
  return has(index, Occupancy::toMask(occupancy_type));
}

inline bool ClassifiedMap::has(const Index3D& index,
                               Occupancy::Mask occupancy_mask) const {
  return has(OctreeIndex{0, index}, occupancy_mask);
}

inline bool ClassifiedMap::has(const OctreeIndex& index,
                               Occupancy::Mask occupancy_mask) const {
  return query_cache_.has(index, occupancy_mask, block_map_);
}

inline bool ClassifiedMap::isFully(const Index3D& index,
                                   Occupancy::Id occupancy_type) const {
  return isFully(OctreeIndex{0, index}, occupancy_type);
}

inline bool ClassifiedMap::isFully(const OctreeIndex& index,
                                   Occupancy::Id occupancy_type) const {
  return isFully(index, Occupancy::toMask(occupancy_type));
}

inline bool ClassifiedMap::isFully(const Index3D& index,
                                   Occupancy::Mask occupancy_mask) const {
  return isFully(OctreeIndex{0, index}, occupancy_mask);
}

inline bool ClassifiedMap::isFully(const OctreeIndex& index,
                                   Occupancy::Mask occupancy_mask) const {
  return query_cache_.isFully(index, occupancy_mask, block_map_);
}

inline void ClassifiedMap::forEachLeafMatching(
    Occupancy::Id occupancy_type, IndexedLeafVisitorFunction visitor_fn,
    IndexElement termination_height) const {
  forEachLeafMatching(Occupancy::toMask(occupancy_type), std::move(visitor_fn),
                      termination_height);
}

inline Occupancy::Mask ClassifiedMap::NodeData::occupancyMask() const {
  return Occupancy::toMask(has_free.any(), has_occupied.any(),
                           has_unobserved.any());
}

inline Occupancy::Mask ClassifiedMap::NodeData::childOccupancyMask(
    NdtreeIndexRelativeChild child_idx) const {
  return Occupancy::toMask(has_free[child_idx], has_occupied[child_idx],
                           has_unobserved[child_idx]);
}

inline bool ClassifiedMap::hasBlock(const Index3D& block_index) const {
  return getBlock(block_index);
}

inline const ClassifiedMap::Block* ClassifiedMap::getBlock(
    const Index3D& block_index) const {
  return query_cache_.getBlock(block_index, block_map_);
}

inline std::pair<const ClassifiedMap::Node*, ClassifiedMap::HeightType>
ClassifiedMap::getNodeOrAncestor(const OctreeIndex& index) const {
  return query_cache_.getNodeOrAncestor(index, block_map_);
}

inline const ClassifiedMap::Node* ClassifiedMap::getNode(
    const OctreeIndex& index) const {
  return getNodeOrAncestor(index).first;
}

inline bool ClassifiedMap::hasValue(const OctreeIndex& index) const {
  return getValue(index).has_value();
}

inline std::optional<Occupancy::Mask> ClassifiedMap::getValue(
    const OctreeIndex& index) const {
  return getValueOrAncestor(index).first;
}

template <typename IndexedBlockVisitor>
void ClassifiedMap::forEachBlock(IndexedBlockVisitor visitor_fn) const {
  block_map_.forEachBlock(visitor_fn);
}

inline const ClassifiedMap::Block* ClassifiedMap::QueryCache::getBlock(
    const Index3D& new_block_index,
    const ClassifiedMap::BlockHashMap& block_map) {
  if (new_block_index != block_index) {
    block_index = new_block_index;
    height = tree_height;
    block = block_map.getBlock(new_block_index);
    if (block) {
      node_stack[tree_height] = &block->getRootNode();
    }
  }
  return block;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
