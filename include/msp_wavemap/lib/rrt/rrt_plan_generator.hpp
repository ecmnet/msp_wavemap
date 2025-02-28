#pragma once
#include <wavemap/core/common.h>
#include <iostream>
#include <random>

namespace msp
{

    class RRTPlanGenerator {

        public:

        explicit RRTPlanGenerator(std::shared_ptr<msp::ESDF> esdf) : esdf_(esdf) {

        }

        private:

        std::shared_ptr<msp::ESDF> esdf_;

        Point3D randomSample() {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_real_distribution<double> disX(0, 10.0);
            std::uniform_real_distribution<double> disY(0, 10.0);
            std::uniform_real_distribution<double> disZ(0, 5.0);
            return {disX(gen), disY(gen), disZ(gen)};
        }

    };
}