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
        public simcars::causal::ABinaryEndogenousVariable<structures::stl::STLStackArray<RewardFWDCarActionPair>,
        FWDCarOutcomeActionPairs, FWDCarRewardParameters>
{
    IFWDCarRewardCalc const *fwd_car_reward_calculator;

public:
    CalcFWDCarActionOutcomeRewardVariable(
            simcars::causal::IEndogenousVariable<FWDCarOutcomeActionPairs> *endogenous_parent,
            simcars::causal::IVariable<FWDCarRewardParameters> *other_parent,
            IFWDCarRewardCalc const *fwd_car_reward_calculator);

    bool get_value(structures::stl::STLStackArray<RewardFWDCarActionPair> &val) const override;

    bool set_value(structures::stl::STLStackArray<RewardFWDCarActionPair> const &val) override;
};

}
}
}
}
