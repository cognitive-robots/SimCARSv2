#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped_outcome.hpp>
#include <ori/simcars/agents/ped_reward_parameters.hpp>
#include <ori/simcars/agents/ped_reward_calc_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class CalcPedActionOutcomeRewardVariable :
        public simcars::causal::ATernaryEndogenousVariable<RewardPedOutcomeActionTuples,
        PedOutcomeActionPairs, PedTask, PedRewardParameters>
{
    IPedRewardCalc const *ped_reward_calculator;

public:
    CalcPedActionOutcomeRewardVariable(
            simcars::causal::IEndogenousVariable<PedOutcomeActionPairs> *endogenous_parent_1,
            simcars::causal::IEndogenousVariable<PedTask> *endogenous_parent_2,
            simcars::causal::IVariable<PedRewardParameters> *other_parent,
            IPedRewardCalc const *ped_reward_calculator);

    bool get_value(RewardPedOutcomeActionTuples &val) const override;

    bool set_value(RewardPedOutcomeActionTuples const &val) override;
};

}
}
}
}
