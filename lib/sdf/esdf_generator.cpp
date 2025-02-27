
#include <msp_wavemap/lib/sdf/esdf_generator.hpp>
#include <omp.h> 

#include <wavemap/core/common.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>

using namespace msp;

void ESDFGenerator::generate_fast_sweep(const Point3D &reference)
{

    auto occupied_cells = getOccupiedCells(reference);

    if(occupied_cells.empty())
        return;

    std::fill(esdf_grid_.begin(), esdf_grid_.end(), std::numeric_limits<float>::max());

    // std::cout << occupied_cells.size() <<std::endl;

    //prepare esd_grid_ with occupied cells
    for (const auto &cell : occupied_cells)
    {
        const auto index = props_.world_to_index(cell, reference);
        esdf_grid_[index] = -1.0f;
    }

    const int centerX = props_.getSizeX() / 2;
    const int centerY = props_.getSizeY() / 2;
    const int centerZ = props_.getSizeZ() / 2;

    //  for (int sweep = 0; sweep < 6; ++sweep) // Why multiple passes original 6
    //  { 
        for (int dz : {-1, 1})
        { // Sweep in Z direction
            for (int dy : {-1, 1})
            { // Sweep in Y direction
                for (int dx : {-1, 1})
                { // Sweep in X direction
                    for (int z = -centerZ + 1; z < centerZ; ++z)
                    {
                        for (int y = -centerY + 1; y < centerY; ++y)
                        {
                            for (int x = -centerX + 1; x < centerX; ++x)
                            {

                                int gridX = x + centerX;
                                int gridY = y + centerY;
                                int gridZ = z + centerZ;

                                float minDist = esdf_grid_[props_.index(gridX, gridY, gridZ)];
                                minDist = std::min(minDist, esdf_grid_[props_.index(gridX + dx, gridY, gridZ)] + 1.0f);
                                minDist = std::min(minDist, esdf_grid_[props_.index(gridX, gridY + dy, gridZ)] + 1.0f);
                                minDist = std::min(minDist, esdf_grid_[props_.index(gridX, gridY, gridZ + dz)] + 1.0f);

                                esdf_grid_[props_.index(gridX, gridY, gridZ)] = minDist;
                            }
                        }
                    }
                }
            }
        }
    //   }
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
