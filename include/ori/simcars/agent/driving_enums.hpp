#pragma once

#include <magic_enum.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

enum class DrivingAgentClass
{
    UNKNOWN = -1,
    CAR = 0,
    VAN = 1,
    TRAM = 2,
    BUS = 3,
    TRUCK = 4,
    EMERGENCY_VEHICLE = 5,
    BICYCLE = 6,
    MOTORCYCLE = 7,
    PEDESTRIAN = 8,
    ANIMAL = 9,
    OTHER_VEHICLE = 10
};

}

inline std::ostream& operator <<(std::ostream& output_stream, const agent::DrivingAgentClass& driving_agent_class)
{
    return output_stream << magic_enum::enum_name(driving_agent_class);
}

}
}
