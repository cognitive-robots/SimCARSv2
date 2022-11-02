
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>

#include <neargye/magic_enum/magic_enum.hpp>

#include <string>
#include <exception>
#include <cmath>
#include <cstdlib>

namespace ori
{
namespace simcars
{
namespace geometry
{

std::weak_ptr<TrigBuff> TrigBuff::instance = std::weak_ptr<TrigBuff>();

TrigBuff::TrigBuff(size_t num_bins, AngleType default_angle_type)
    : num_bins(num_bins), default_angle_type(default_angle_type), radian_multi(num_bins / (2.0f * M_PI)), degree_multi(num_bins / 360.0f)
{
    if (num_bins < 1)
    {
        throw std::invalid_argument("Number of bins for trigonometry buffer initialisation should be at least 1, "
                                    "the value passed was '" + std::to_string(num_bins) + "'");
    }

    if (!magic_enum::enum_contains(default_angle_type))
    {
        throw std::invalid_argument("Default angle type for trigonometry buffer initialisation was invalid, "
                                    "the value passed (in integer form) was '"
                                    + std::to_string(magic_enum::enum_integer(default_angle_type)) + "'");
    }

    sin_bins = std::unique_ptr<FP_DATA_TYPE[]>(new FP_DATA_TYPE[num_bins]);
    cos_bins = std::unique_ptr<FP_DATA_TYPE[]>(new FP_DATA_TYPE[num_bins]);
    tan_bins = std::unique_ptr<FP_DATA_TYPE[]>(new FP_DATA_TYPE[num_bins]);

    FP_DATA_TYPE angle;
    size_t i;
    for (i = 0; i < num_bins; ++i)
    {
        angle = i * (2.0f * M_PI) / num_bins;
        sin_bins[i] = sin(angle);
        cos_bins[i] = cos(angle);
        tan_bins[i] = tan(angle);
    }
}

std::shared_ptr<const TrigBuff> TrigBuff::init_instance(size_t num_bins, AngleType default_angle_type)
{
    std::shared_ptr<TrigBuff> instance(new TrigBuff(num_bins, default_angle_type));
    TrigBuff::instance = instance;
    return instance;
}

std::shared_ptr<const TrigBuff> TrigBuff::get_instance()
{
    std::shared_ptr<TrigBuff> instance = TrigBuff::instance.lock();
    if (instance != nullptr)
    {
        return instance;
    }
    else
    {
        throw std::runtime_error("A trigonometry buffer instance has not been initialised");
    }
}

FP_DATA_TYPE TrigBuff::get_sin(FP_DATA_TYPE angle) const
{
    if (std::isinf(angle))
    {
        throw std::invalid_argument("Angle is infinite");
    }

    switch (default_angle_type)
    {
    case AngleType::RADIANS:
        if (angle < 0.0f) return get_sin(angle + (2.0f * M_PI) * (int(-angle / (2.0f * M_PI)) + 1.0f));
        return sin_bins[int(angle * radian_multi) % num_bins];

    case AngleType::DEGREES:
        if (angle < 0.0f) return get_sin(angle + 360.0f * (int(-angle / 360.0f) + 1.0f));
        return sin_bins[int(angle * degree_multi) % num_bins];

    default:
        throw utils::NotImplementedException();
    }
}

FP_DATA_TYPE TrigBuff::get_sin(FP_DATA_TYPE angle, AngleType angle_type) const
{
    if (std::isinf(angle))
    {
        throw std::invalid_argument("Angle is infinite");
    }

    switch (angle_type)
    {
    case AngleType::RADIANS:
        if (angle < 0.0f) return get_sin(angle + (2.0f * M_PI) * (int(-angle / (2.0f * M_PI)) + 1.0f), angle_type);
        return sin_bins[int(angle * radian_multi) % num_bins];

    case AngleType::DEGREES:
        if (angle < 0.0f) return get_sin(angle + 360.0f * (int(-angle / 360.0f) + 1.0f), angle_type);
        return sin_bins[int(angle * degree_multi) % num_bins];

    default:
        throw std::invalid_argument("Input angle type (in integer form) '" + std::to_string(magic_enum::enum_integer(angle_type)) + "' is not a valid angle type for a sine calculation");
    }
}

FP_DATA_TYPE TrigBuff::get_cos(FP_DATA_TYPE angle) const
{
    if (std::isinf(angle))
    {
        throw std::invalid_argument("Angle is infinite");
    }

    switch (default_angle_type)
    {
    case AngleType::RADIANS:
        if (angle < 0.0f) return get_cos(angle + (2.0f * M_PI) * (int(-angle / (2.0f * M_PI)) + 1.0f));
        return cos_bins[int(angle * radian_multi) % num_bins];

    case AngleType::DEGREES:
        if (angle < 0.0f) return get_cos(angle + 360.0f * (int(-angle / 360.0f) + 1.0f));
        return cos_bins[int(angle * degree_multi) % num_bins];

    default:
        throw utils::NotImplementedException();
    }
}

FP_DATA_TYPE TrigBuff::get_cos(FP_DATA_TYPE angle, AngleType angle_type) const
{
    if (std::isinf(angle))
    {
        throw std::invalid_argument("Angle is infinite");
    }

    switch (angle_type)
    {
    case AngleType::RADIANS:
        if (angle < 0.0f) return get_cos(angle + (2.0f * M_PI) * (int(-angle / (2.0f * M_PI)) + 1.0f), angle_type);
        return cos_bins[int(angle * radian_multi) % num_bins];

    case AngleType::DEGREES:
        if (angle < 0.0f) return get_cos(angle + 360.0f * (int(-angle / 360.0f) + 1.0f), angle_type);
        return cos_bins[int(angle * degree_multi) % num_bins];

    default:
        throw std::invalid_argument("Input angle type (in integer form) '" + std::to_string(magic_enum::enum_integer(angle_type)) + "' is not a valid angle type for a cosine calculation");
    }
}

FP_DATA_TYPE TrigBuff::get_tan(FP_DATA_TYPE angle) const
{
    if (std::isinf(angle))
    {
        throw std::invalid_argument("Angle is infinite");
    }

    switch (default_angle_type)
    {
    case AngleType::RADIANS:
        if (angle < 0.0f) return get_tan(angle + (2.0f * M_PI) * (int(-angle / (2.0f * M_PI)) + 1.0f));
        return tan_bins[int(angle * radian_multi) % num_bins];

    case AngleType::DEGREES:
        if (angle < 0.0f) return get_tan(angle + 360.0f * (int(-angle / 360.0f) + 1.0f));
        return tan_bins[int(angle * degree_multi) % num_bins];

    default:
        throw utils::NotImplementedException();
    }
}

FP_DATA_TYPE TrigBuff::get_tan(FP_DATA_TYPE angle, AngleType angle_type) const
{
    if (std::isinf(angle))
    {
        throw std::invalid_argument("Angle is infinite");
    }

    switch (angle_type)
    {
    case AngleType::RADIANS:
        if (angle < 0.0f) return get_tan(angle + (2.0f * M_PI) * (int(-angle / (2.0f * M_PI)) + 1.0f), angle_type);
        return tan_bins[int(angle * radian_multi) % num_bins];

    case AngleType::DEGREES:
        if (angle < 0.0f) return get_tan(angle + 360.0f * (int(-angle / 360.0f) + 1.0f), angle_type);
        return tan_bins[int(angle * degree_multi) % num_bins];

    default:
        throw std::invalid_argument("Input angle type (in integer form) '" + std::to_string(magic_enum::enum_integer(angle_type)) + "' is not a valid angle type for a tangent calculation");
    }
}

RotMat TrigBuff::get_rot_mat(FP_DATA_TYPE angle) const
{
    RotMat rot_mat;
    FP_DATA_TYPE sin_a = get_sin(angle);
    FP_DATA_TYPE cos_a = get_cos(angle);
    rot_mat << cos_a, -sin_a, sin_a, cos_a;
    return rot_mat;
}

RotMat TrigBuff::get_rot_mat(FP_DATA_TYPE angle, AngleType angle_type) const
{
    RotMat rot_mat;
    FP_DATA_TYPE sin_a = get_sin(angle, angle_type);
    FP_DATA_TYPE cos_a = get_cos(angle, angle_type);
    rot_mat << cos_a, -sin_a, sin_a, cos_a;
    return rot_mat;
}

}
}
}
