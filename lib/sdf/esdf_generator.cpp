
#include <msp_wavemap/lib/sdf/esdf_generator.hpp>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>

using namespace msp;

void ESDFGenerator::generate_fast_sweep(const Point3D &reference)
{

    auto occupied_cells = getOccupiedCells(reference);

    if (occupied_cells.empty())
        return;

    auto esdf_grid_ = esdf_->getData();

    std::fill(esdf_grid_->begin(), esdf_grid_->end(), std::numeric_limits<float>::max());

    // prepare esd_grid_ with occupied cells
    for (const auto &cell : occupied_cells)
    {
        const auto index = esdf_->world_to_index(cell, reference);
        (*esdf_grid_)[index] = -1.0f;
    }

    const Eigen::Vector3i center(int(esdf_->getSizeX()) / 2, int(esdf_->getSizeY()) / 2, int(esdf_->getSizeZ()) / 2);

    for (int dz : {-1, 1})
    { // Sweep in Z direction
        for (int dy : {-1, 1})
        { // Sweep in Y direction
            for (int dx : {-1, 1})
            { // Sweep in X direction
                if (thread_pool_)
                    thread_pool_->add_task([this, center, dz, dy, dx](){ this->processESDFChunk(center, dz,dy,dx); });
                else
                    processESDFChunk(center, dz,dy,dx);
            }
            if (thread_pool_)
                thread_pool_->wait_all();    
        }
    }
}


void ESDFGenerator::processESDFChunk(Eigen::Vector3i center, int dz, int dy, int dx)
{
            auto esdf_grid_ = esdf_->getData();
            for (int z = -center.z() + 1; z < center.z(); ++z)
            {
                for (int y = -center.y() + 1; y < center.y(); ++y)
                {
                    for (int x = -center.x() + 1; x < center.x(); ++x)
                    {
                        const int gridX = x + center.x();
                        const int gridY = y + center.y();
                        const int gridZ = z + center.z();

                        float minDist = (*esdf_grid_)[esdf_->index(gridX, gridY, gridZ)];
                        minDist = std::min(minDist, (*esdf_grid_)[esdf_->index(gridX + dx, gridY, gridZ)] + 1.0f);
                        minDist = std::min(minDist, (*esdf_grid_)[esdf_->index(gridX, gridY + dy, gridZ)] + 1.0f);
                        minDist = std::min(minDist, (*esdf_grid_)[esdf_->index(gridX, gridY, gridZ + dz)] + 1.0f);

                        (*esdf_grid_)[esdf_->index(gridX, gridY, gridZ)] = minDist;
                    }
                }
            }
}

std::vector<Point3D> ESDFGenerator::getOccupiedCells(const Point3D &reference)
{
    std::vector<Point3D> list;

    const Point3D lower_bound{reference - dimensions_ / 2.0f};
    const Point3D upper_bound{reference + dimensions_ / 2.0f};

    if (const auto *hashed_wavelet_octree =
            dynamic_cast<const HashedWaveletOctree *>(map_.get());
        hashed_wavelet_octree)
    {

        const FloatingPoint min_cell_width = hashed_wavelet_octree->getMinCellWidth();
        const Index3D min_corner_index =
            convert::pointToFloorIndex(lower_bound,
                                       1.f / min_cell_width);
        const wavemap::Index3D max_corner_index =
            convert::pointToCeilIndex(upper_bound,
                                      1.f / min_cell_width);

        QueryAccelerator query_accelerator(*hashed_wavelet_octree);
        for (const auto &query_index :
             wavemap::Grid<3>(min_corner_index, max_corner_index))
        {
            const FloatingPoint occupancy_log_odds = query_accelerator.getCellValue(query_index);
            if (occupancy_log_odds > 0.5)
            {
                const auto index = OctreeIndex{0, query_index};
                const Point3D block = convert::indexToCenterPoint(index.position, min_cell_width);
                list.emplace_back(block);
            }
        }
    }

    return list;
}
