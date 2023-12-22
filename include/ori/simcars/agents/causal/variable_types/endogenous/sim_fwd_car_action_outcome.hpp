#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/fwd_car_outcome_sim_interface.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>
#include <ori/simcars/agents/fwd_car_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class SimFWDCarActionOutcomeVariable :
        public simcars::causal::ABinaryEndogenousVariable<
        structures::stl::STLStackArray<FWDCarOutcomeActionPair>,
        structures::stl::STLStackArray<FWDCarAction>, FWDCarSimParameters>
{
    IFWDCarOutcomeSim const *fwd_car_outcome_sim;

public:
    SimFWDCarActionOutcomeVariable(simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<FWDCarAction>> const *endogenous_parent,
                           simcars::causal::IVariable<FWDCarSimParameters> const *other_parent,
                           IFWDCarOutcomeSim const *fwd_car_outcome_sim);

    bool get_value(structures::stl::STLStackArray<FWDCarOutcomeActionPair> &val) const override;
};

}
}
}
}
