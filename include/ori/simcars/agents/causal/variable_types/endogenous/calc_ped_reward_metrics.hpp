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

class CalcPedRewardMetricsVariable : public simcars::causal::ATernaryEndogenousVariable<PedRewards,
        PedOutcomeActionPair, PedTask, PedRewardParameters>
{
    IPedRewardCalc const *ped_reward_calculator;

public:
    CalcPedRewardMetricsVariable(
            simcars::causal::IEndogenousVariable<PedOutcomeActionPair> *endogenous_parent_1,
            simcars::causal::IEndogenousVariable<PedTask> *endogenous_parent_2,
            simcars::causal::IVariable<PedRewardParameters> *other_parent,
            IPedRewardCalc const *ped_reward_calculator);

    bool get_value(PedRewards &val) const override;

    bool set_value(PedRewards const &val) override;
};

}
}
}
}
