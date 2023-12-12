#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>
#include <ori/simcars/agents/fwd_car_reward_parameters.hpp>
#include <ori/simcars/agents/fwd_car_reward_calc_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class CalcFWDCarActionOutcomeRewardVariable :
        public simcars::causal::ABinaryEndogenousVariable<structures::stl::STLStackArray<RewardFWDCarActionPair>, structures::stl::STLStackArray<FWDCarOutcomeActionPair>, FWDCarRewardParameters>
{
    IFWDCarRewardCalc const *fwd_car_reward_calculator;

public:
    CalcFWDCarActionOutcomeRewardVariable(simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<FWDCarOutcomeActionPair>> const *endogenous_parent,
                                  simcars::causal::IVariable<FWDCarRewardParameters> const *other_parent,
                                  IFWDCarRewardCalc const *fwd_car_reward_calculator);

    structures::stl::STLStackArray<RewardFWDCarActionPair> get_value() const override;
};

}
}
}
}
