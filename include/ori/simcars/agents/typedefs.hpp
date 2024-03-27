#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/goal.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>

#include <utility>

namespace ori
{
namespace simcars
{
namespace agents
{

typedef std::pair<FWDCarOutcome, FWDCarAction> FWDCarOutcomeActionPair;

typedef structures::stl::STLStackArray<FWDCarOutcomeActionPair> FWDCarOutcomeActionPairs;

typedef std::pair<FP_DATA_TYPE, FWDCarAction> RewardFWDCarActionPair;

typedef structures::stl::STLStackArray<std::pair<FP_DATA_TYPE, FWDCarAction>> RewardFWDCarActionPairs;

typedef std::tuple<FP_DATA_TYPE, FWDCarOutcome, FWDCarAction> RewardFWDCarOutcomeActionTuple;

typedef structures::stl::STLStackArray<std::tuple<FP_DATA_TYPE, FWDCarOutcome, FWDCarAction>> RewardFWDCarOutcomeActionTuples;

template <typename T>
using TimeGoalPair = std::pair<temporal::Time, Goal<T>>;

typedef std::pair<temporal::Time, FWDCarAction> TimeFWDCarActionPair;

}
}
}
