#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>
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

class GenerateFWDCarAction : public simcars::causal::ATernaryEndogenousVariable<
        structures::stl::STLStackArray<FWDCarAction>,
        structures::stl::STLStackArray<temporal::Time>,
        structures::stl::STLStackArray<FP_DATA_TYPE>, structures::stl::STLStackArray<uint64_t>>
{
public:
    using simcars::causal::ATernaryEndogenousVariable<
    structures::stl::STLStackArray<FWDCarAction>,
    structures::stl::STLStackArray<temporal::Time>,
    structures::stl::STLStackArray<FP_DATA_TYPE>, structures::stl::STLStackArray<uint64_t>
    >::ATernaryEndogenousVariable;

    structures::stl::STLStackArray<FWDCarAction> get_value() const override;
};

}
}
}
}
