#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <Eigen/Core>

namespace ori
{
namespace simcars
{
namespace geometry
{

typedef Eigen::Matrix<FP_DATA_TYPE, 2, 1> Vec;
typedef Eigen::Matrix<FP_DATA_TYPE, 2, Eigen::Dynamic> Vecs;
typedef Eigen::Matrix<FP_DATA_TYPE, 2, 2> RotMat;

}
}
}
