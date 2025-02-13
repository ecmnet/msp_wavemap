#ifndef WAVEMAP_CORE_COMMON_H_
#define WAVEMAP_CORE_COMMON_H_

#include <limits>
#include <vector>

#include <Eigen/Eigen>
#include <wavemap/3rd_party/minkindr/quat-transformation.h>

#include "wavemap/core/utils/data/constants.h"

namespace wavemap {
using FloatingPoint = float;

template <typename T>
struct Bounds {
  T lower = std::numeric_limits<T>::max();
  T upper = std::numeric_limits<T>::lowest();
};

template <typename T>
using MatrixT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
using Matrix = MatrixT<FloatingPoint>;

using IndexElement = int;
template <int dim>
using Index = Eigen::Matrix<IndexElement, dim, 1>;
using Index2D = Index<2>;
using Index3D = Index<3>;

using LinearIndex = size_t;
using MortonIndex = uint64_t;

template <int dim>
using Vector = Eigen::Matrix<FloatingPoint, dim, 1>;
using Vector2D = Vector<2>;
using Vector3D = Vector<3>;

template <int dim>
using Point = Vector<dim>;
using Point2D = Point<2>;
using Point3D = Point<3>;

using Transformation3D =
    kindr::minimal::QuatTransformationTemplate<FloatingPoint>;

using Rotation3D = Transformation3D::Rotation;

template <typename T>
inline constexpr int dim_v = T::RowsAtCompileTime;
template <>
inline constexpr int dim_v<Transformation3D> = 3;
template <>
inline constexpr int dim_v<Rotation3D> = 3;

using ImageCoordinates = Vector2D;
struct SensorCoordinates {
  ImageCoordinates image;
  FloatingPoint depth;
};

constexpr auto kEpsilon = constants<FloatingPoint>::kEpsilon;
constexpr auto kNaN = std::numeric_limits<FloatingPoint>::quiet_NaN();

constexpr auto kPi = constants<FloatingPoint>::kPi;
constexpr auto kTwoPi = constants<FloatingPoint>::kTwoPi;
constexpr auto kHalfPi = constants<FloatingPoint>::kHalfPi;
constexpr auto kQuarterPi = constants<FloatingPoint>::kQuarterPi;
}  // namespace wavemap

#endif  // WAVEMAP_CORE_COMMON_H_
