#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <Eigen/Core>

#define GOLDEN_RATIO_MAGIC_NUM 0x9e3779b9

namespace ori
{
namespace simcars
{
namespace geometry
{

typedef Eigen::Matrix<FP_DATA_TYPE, 2, 1> Vec;
typedef std::pair<Vec, Vec> VecPair;
typedef Eigen::Matrix<FP_DATA_TYPE, 2, Eigen::Dynamic> Vecs;
typedef Eigen::Matrix<FP_DATA_TYPE, 2, 2> RotMat;

class VecHasher
{
    std::hash<FP_DATA_TYPE> hasher;

public:
    std::size_t operator()(Vec const &key) const
    {
        size_t key_hash = hasher(key.x());
        key_hash ^= hasher(key.y()) + GOLDEN_RATIO_MAGIC_NUM + (key_hash << 6) + (key_hash >> 2);
        return key_hash;
    }
};

}
}
}
