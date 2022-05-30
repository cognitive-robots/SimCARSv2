#pragma once

#include <ori/simcars/geometry/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IAgentStateHolder
{
public:
    enum class Class
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

    enum class Status
    {
        UNKNOWN = -1,
        NOMINAL = 10,
        WARNING = 20,
        ERROR = 30,
        FATAL = 40
    };

    struct State
    {
        IAgentStateHolder::Status status;
        geometry::Vec position;
        FP_DATA_TYPE rotation;
        geometry::Vec linear_velocity;
        FP_DATA_TYPE angular_velocity;
        geometry::Vec linear_acceleration;
        FP_DATA_TYPE angular_acceleration;

        State(const geometry::Vec& position, FP_DATA_TYPE rotation,
              const geometry::Vec& linear_velocity, FP_DATA_TYPE angular_velocity,
              const geometry::Vec& linear_acceleration, FP_DATA_TYPE angular_acceleration,
              IAgentStateHolder::Status status = IAgentStateHolder::Status::UNKNOWN)
            : position(position), rotation(rotation), linear_velocity(linear_velocity),
              angular_velocity(angular_velocity), linear_acceleration(linear_acceleration),
              angular_acceleration(angular_acceleration), status(status) {}
        State(const State& state)
            : State(state.position, state.rotation, state.linear_velocity, state.angular_velocity, state.linear_acceleration,
                    state.angular_acceleration, state.status) {}
        State()
            : State(geometry::Vec::Zero(), 0.0f,
                    geometry::Vec::Zero(), 0.0f,
                    geometry::Vec::Zero(), 0.0f) {}
        virtual ~State() = default;

        virtual bool operator ==(const IAgentStateHolder::State& state) const
        {
            return this->position == state.position
                    && this->rotation == state.rotation
                    && this->linear_velocity == state.linear_velocity
                    && this->angular_velocity == state.angular_velocity
                    && this->linear_acceleration == state.linear_acceleration
                    && this->angular_acceleration == state.angular_acceleration;
        }
    };

    virtual ~IAgentStateHolder() = default;
};

}
}
}
