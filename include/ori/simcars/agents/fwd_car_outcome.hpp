#pragma once

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarOutcome
{
};

bool operator==(FWDCarOutcome const &outcome_1, FWDCarOutcome const &outcome_2)
{
    return &outcome_1 == &outcome_2;
}

}
}
}
