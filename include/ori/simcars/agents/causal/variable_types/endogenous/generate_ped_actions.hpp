#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class GeneratePedActionsVariable : public simcars::causal::ABinaryEndogenousVariable<
        structures::stl::STLStackArray<PedAction>, structures::stl::STLStackArray<temporal::Time>,
        structures::stl::STLStackArray<uint64_t>>
{
public:
    using simcars::causal::ABinaryEndogenousVariable<structures::stl::STLStackArray<PedAction>,
    structures::stl::STLStackArray<temporal::Time>, structures::stl::STLStackArray<uint64_t>
    >::ABinaryEndogenousVariable;

    bool get_value(structures::stl::STLStackArray<PedAction> &val) const override;

    bool set_value(structures::stl::STLStackArray<PedAction> const &val) override;
};

}
}
}
}
