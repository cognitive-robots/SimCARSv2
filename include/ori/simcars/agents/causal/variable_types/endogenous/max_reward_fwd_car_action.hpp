#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class MaxRewardFWDCarAction :
        public simcars::causal::AUnaryEndogenousVariable<FWDCarAction, structures::stl::STLStackArray<RewardFWDCarActionPair>>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<FWDCarAction, structures::stl::STLStackArray<RewardFWDCarActionPair>>::AUnaryEndogenousVariable;

    FWDCarAction get_value() const override;
};

}
}
}
}
