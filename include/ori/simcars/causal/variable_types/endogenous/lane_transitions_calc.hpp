#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class LaneTransitionsCalcVariable : public ABinaryEndogenousVariable<FP_DATA_TYPE,
        structures::stl::STLStackArray<uint64_t>,
        structures::stl::STLStackArray<uint64_t>>
{
    map::IMap const *map;

public:
    LaneTransitionsCalcVariable(
            IEndogenousVariable<structures::stl::STLStackArray<uint64_t>> const *endogenous_parent,
            IVariable<structures::stl::STLStackArray<uint64_t>> const *other_parent,
            map::IMap const *map);

    FP_DATA_TYPE get_value() const override;
};

}
}
}
