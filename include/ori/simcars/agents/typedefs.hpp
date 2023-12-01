#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/declarations.hpp>

#include <utility>

namespace ori
{
namespace simcars
{
namespace agents
{

typedef std::pair<FWDCarOutcome, FWDCarAction> FWDCarOutcomeActionPair;

typedef std::pair<FP_DATA_TYPE, FWDCarAction> RewardFWDCarActionPair;

}
}
}
