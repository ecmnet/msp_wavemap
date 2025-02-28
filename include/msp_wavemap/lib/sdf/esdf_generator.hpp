
#pragma once
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <msp_wavemap/lib/sdf/esdf.hpp>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <vector>
#include <iostream>
#include <cmath>

#define MAX_DISTANCE 99.0f

using namespace wavemap;

namespace msp
{

    class ESDFGenerator
    {
    public:
        explicit ESDFGenerator(MapBase::Ptr map, Point3D dimensions,
                               std::shared_ptr<ThreadPool> thread_pool  = nullptr) 
                               : map_(map), dimensions_(dimensions), thread_pool_(thread_pool)
        {
        }

        /*
        * Generate a fixed 3D ESDF with size 10x10x5 
        */
        void generate_fast_sweep(const Point3D &reference);

        /*
        * Returns an ESDF gradiant at a certain pont in world coordinates
        */

        std::shared_ptr<msp::ESDF>  getESDF() { return esdf_; }


    private:

        std::vector<Point3D> getOccupiedCells(const Point3D &ref);
        std::shared_ptr<ThreadPool> thread_pool_;

        std::shared_ptr<msp::ESDF> esdf_ = std::make_unique<msp::ESDF>(dimensions_,0.2f);

        MapBase::Ptr map_;
        Point3D dimensions_;

        void processESDFChunk(Eigen::Vector3i center, int dz, int dy, int dx);
        
        
        
    };
}