#pragma once

#include <ori/simcars/geometry/enums.hpp>
#include <ori/simcars/geometry/typedefs.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <memory>

namespace ori
{
namespace simcars
{
namespace geometry
{

class TrigBuff
{
    static std::weak_ptr<TrigBuff> instance;

    const size_t num_bins;
    const AngleType default_angle_type;
    const FP_DATA_TYPE radian_multi;
    const FP_DATA_TYPE degree_multi;
    std::unique_ptr<FP_DATA_TYPE[]> sin_bins;
    std::unique_ptr<FP_DATA_TYPE[]> cos_bins;
    std::unique_ptr<FP_DATA_TYPE[]> tan_bins;

    TrigBuff(size_t num_bins, AngleType default_angle_type);
    TrigBuff(const TrigBuff&) = delete;
    TrigBuff(TrigBuff&&) = delete;

public:
    static std::shared_ptr<const TrigBuff> init_instance(size_t num_bins, AngleType default_angle_type);
    static std::shared_ptr<const TrigBuff> get_instance();

    virtual ~TrigBuff() = default;

    FP_DATA_TYPE get_sin(FP_DATA_TYPE angle) const;
    FP_DATA_TYPE get_sin(FP_DATA_TYPE angle, AngleType angle_type) const;
    FP_DATA_TYPE get_cos(FP_DATA_TYPE angle) const;
    FP_DATA_TYPE get_cos(FP_DATA_TYPE angle, AngleType angle_type) const;
    FP_DATA_TYPE get_tan(FP_DATA_TYPE angle) const;
    FP_DATA_TYPE get_tan(FP_DATA_TYPE angle, AngleType angle_type) const;
    RotMat get_rot_mat(FP_DATA_TYPE angle) const;
    RotMat get_rot_mat(FP_DATA_TYPE angle, AngleType angle_type) const;
};

}
}
}
