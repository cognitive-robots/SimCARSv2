#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

template <typename T>
struct Goal
{
    T val;
    temporal::Time time;
};

template <typename T>
bool operator==(Goal<T> const &goal_1, Goal<T> const &goal_2)
{
    return goal_1.val == goal_2.val && goal_1.time == goal_2.time;
}

}
}
}
