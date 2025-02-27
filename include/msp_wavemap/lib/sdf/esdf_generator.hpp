
#pragma once
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <msp_wavemap/lib/sdf/esdf.hpp>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <queue>
#include <mutex>  

using namespace wavemap;

namespace msp
{

    class ESDFGenerator
    {
    public:
        explicit ESDFGenerator(MapBase::Ptr map, Point3D dimensions,
                               std::shared_ptr<ThreadPool> thread_pool  = nullptr) 
                               : map_(map), dimensions_(dimensions), props_(dimensions, 0.2f), thread_pool_(thread_pool)
        {
            esdf_grid_.resize(props_.getSizeX() * props_.getSizeY() * props_.getSizeZ(), std::numeric_limits<float>::max());


        }

     //   void generate_dijkstra(const Point3D &reference);
        void generate_fast_sweep(const Point3D &reference);

        std::vector<float>* getData()
        {
            return &esdf_grid_;
        }

        msp::ESDFProperties* getProperties()
        {
            return &props_;
        }


    private:

        std::vector<Point3D> getOccupiedCells(const Point3D &ref);
        std::shared_ptr<ThreadPool> thread_pool_;

        std::mutex esdf_mutex; 
        msp::ESDFProperties props_;
        MapBase::Ptr map_;
        Point3D dimensions_;
        std::vector<float> esdf_grid_;

        void processESDFChunk(Eigen::Vector3i center, int dz, int dy, int dx);
        
        
        
    };
}