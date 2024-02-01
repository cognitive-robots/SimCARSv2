#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/goal.hpp>

#include <utility>

namespace ori
{
namespace simcars
{
namespace agents
{

typedef std::pair<FWDCarOutcome, FWDCarAction> FWDCarOutcomeActionPair;

typedef std::pair<FP_DATA_TYPE, FWDCarAction> RewardFWDCarActionPair;

template <typename T>
using TimeGoalPair = std::pair<temporal::Time, Goal<T>>;

typedef std::pair<temporal::Time, FWDCarAction> TimeFWDCarActionPair;

}
}
}
