#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/ped_outcome_calc_interface.hpp>
#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped_outcome.hpp>
#include <ori/simcars/agents/ped_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class CalcPedActionOutcomeVariable :
        public simcars::causal::AUnaryEndogenousVariable<
        PedOutcomeActionPair, PedSimParameters>
{
    IPedOutcomeCalc const *ped_outcome_calc;

public:
    CalcPedActionOutcomeVariable(simcars::causal::IVariable<PedSimParameters> *parent,
                                 IPedOutcomeCalc const *ped_outcome_calc);

    bool get_value(PedOutcomeActionPair &val) const override;

    bool set_value(PedOutcomeActionPair const &val) override;
};

}
}
}
}
