#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/ped_outcome_sim_interface.hpp>
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

class SimPedActionOutcomeVariable :
        public simcars::causal::ABinaryEndogenousVariable<
        PedOutcomeActionPairs,
        structures::stl::STLStackArray<PedAction>, PedSimParameters>
{
    IPedOutcomeSim const *ped_outcome_sim;

public:
    SimPedActionOutcomeVariable(
            simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<PedAction>> *endogenous_parent,
            simcars::causal::IVariable<PedSimParameters> *other_parent,
            IPedOutcomeSim const *ped_outcome_sim);

    bool get_value(PedOutcomeActionPairs &val) const override;

    bool set_value(PedOutcomeActionPairs const &val) override;
};

}
}
}
}
