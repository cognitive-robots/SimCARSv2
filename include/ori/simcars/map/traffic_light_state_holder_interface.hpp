#pragma once

namespace ori
{
namespace simcars
{
namespace map
{

class ITrafficLightStateHolder
{
public:
    enum class FaceType
    {
        UNKNOWN = -1,
        STANDARD = 0,
        ARROW = 1,
        LEFT = 4,
        LEFT_ARROW = 5,
        RIGHT = 8,
        RIGHT_ARROW = 9,
        UPPER = 16,
        UPPER_LEFT_ARROW = 21,
        UPPER_RIGHT_ARROW = 25,
        UTURN = 32,
        FLASHING = 64
    };
    enum class FaceColour
    {
        UNKNOWN = -1,
        RED = 0,
        YELLOW = 1,
        GREEN = 2
    };

    struct State
    {
        ITrafficLightStateHolder::FaceColour active_face;

        State(ITrafficLightStateHolder::FaceColour active_face = ITrafficLightStateHolder::FaceColour::UNKNOWN)
            : active_face(active_face) {}
        State(State const &state) : State(state.active_face) {}
        virtual ~State() = default;

        virtual bool operator ==(ITrafficLightStateHolder::State const &state) const
        {
            return this->active_face == state.active_face;
        }
    };

    ~ITrafficLightStateHolder() = default;
};

}
}
}
