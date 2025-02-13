#ifndef WAVEMAP_IO_STREAM_CONVERSIONS_H_
#define WAVEMAP_IO_STREAM_CONVERSIONS_H_

#include <istream>
#include <ostream>

#include "wavemap/core/common.h"
#include "wavemap/core/map/cell_types/haar_coefficients.h"
#include "wavemap/core/map/hashed_blocks.h"
#include "wavemap/core/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/map/wavelet_octree.h"
#include "wavemap/io/streamable_types.h"

namespace wavemap::io {
bool mapToStream(const MapBase& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, MapBase::Ptr& map);

bool mapToStream(const HashedBlocks& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedBlocks::Ptr& map);

bool mapToStream(const WaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, WaveletOctree::Ptr& map);

bool mapToStream(const HashedWaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedWaveletOctree::Ptr& map);

bool mapToStream(const HashedChunkedWaveletOctree& map, std::ostream& ostream);
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_STREAM_CONVERSIONS_H_
