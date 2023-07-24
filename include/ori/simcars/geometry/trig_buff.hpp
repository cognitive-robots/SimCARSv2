#pragma once

#include <ori/simcars/geometry/enums.hpp>
#include <ori/simcars/geometry/typedefs.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace ori
{
namespace simcars
{
namespace geometry
{

class TrigBuff
{
    static TrigBuff *instance;

    size_t const num_bins;
    AngleType const default_angle_type;
    FP_DATA_TYPE const radian_multi;
    FP_DATA_TYPE const degree_multi;

    FP_DATA_TYPE *sin_bins;
    FP_DATA_TYPE *cos_bins;
    FP_DATA_TYPE *tan_bins;

    TrigBuff(size_t num_bins, AngleType default_angle_type);
    TrigBuff(TrigBuff const&) = delete;
    TrigBuff(TrigBuff&&) = delete;

public:
    static TrigBuff const* init_instance(size_t num_bins, AngleType default_angle_type);
    static void destroy_instance();
    static TrigBuff const* get_instance()
    {
        if (TrigBuff::instance != nullptr)
        {
            return TrigBuff::instance;
        }
        else
        {
            throw std::runtime_error("A trigonometry buffer instance has not been initialised");
        }
    }

    virtual ~TrigBuff();

    FP_DATA_TYPE wrap(FP_DATA_TYPE angle) const;
    FP_DATA_TYPE wrap(FP_DATA_TYPE angle, AngleType angle_type) const;
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
