#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/goal.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>
#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped_outcome.hpp>

#include <utility>

namespace ori
{
namespace simcars
{
namespace agents
{

template <typename T>
using TimeGoalPair = std::pair<temporal::Time, Goal<T>>;


typedef std::pair<FWDCarOutcome, FWDCarAction> FWDCarOutcomeActionPair;

typedef structures::stl::STLStackArray<FWDCarOutcomeActionPair> FWDCarOutcomeActionPairs;

typedef std::pair<FP_DATA_TYPE, FWDCarAction> RewardFWDCarActionPair;

typedef structures::stl::STLStackArray<std::pair<FP_DATA_TYPE, FWDCarAction>> RewardFWDCarActionPairs;

typedef std::tuple<FP_DATA_TYPE, FWDCarOutcome, FWDCarAction> RewardFWDCarOutcomeActionTuple;

typedef structures::stl::STLStackArray<std::tuple<FP_DATA_TYPE, FWDCarOutcome, FWDCarAction>> RewardFWDCarOutcomeActionTuples;

typedef std::pair<temporal::Time, FWDCarAction> TimeFWDCarActionPair;


typedef std::pair<PedOutcome, PedAction> PedOutcomeActionPair;

typedef structures::stl::STLStackArray<PedOutcomeActionPair> PedOutcomeActionPairs;

typedef std::pair<FP_DATA_TYPE, PedAction> RewardPedActionPair;

typedef structures::stl::STLStackArray<std::pair<FP_DATA_TYPE, PedAction>> RewardPedActionPairs;

typedef std::tuple<FP_DATA_TYPE, PedOutcome, PedAction> RewardPedOutcomeActionTuple;

typedef structures::stl::STLStackArray<std::tuple<FP_DATA_TYPE, PedOutcome, PedAction>> RewardPedOutcomeActionTuples;

typedef std::pair<temporal::Time, PedAction> TimePedActionPair;

}
}
}
