#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class Goal;

}

template <typename T>
std::ostream& operator<<(std::ostream &output_stream, agent::Goal<T> const &goal);

namespace agent
{

template <typename T>
class Goal
{
    T goal_value;
    temporal::Time goal_time;

public:
    Goal() {}
    Goal(T goal_value, temporal::Time goal_time)
        : goal_value(goal_value), goal_time(goal_time) {}

    T const& get_goal_value() const
    {
        return goal_value;
    }
    temporal::Time const& get_goal_time() const
    {
        return goal_time;
    }

    void set_goal_value(T const &goal_value)
    {
        this->goal_value = goal_value;
    }
    void set_goal_time(temporal::Time const &goal_time)
    {
        this->goal_time = goal_time;
    }
};

}

template <typename T>
inline std::ostream& operator<<(std::ostream &output_stream, agent::Goal<T> const &goal)
{
    return output_stream << "Goal(value = " <<
                            std::to_string(goal.get_goal_value()) <<
                            ", time = " << goal.get_goal_time() << ")" <<
                            std::endl;
}

}
}
