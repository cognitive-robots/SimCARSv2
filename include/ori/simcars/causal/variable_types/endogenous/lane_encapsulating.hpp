#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class LaneEncapsulatingVariable : public AUnaryEndogenousVariable<
        structures::stl::STLStackArray<uint64_t>,
        geometry::Vec>
{
    map::IMap const *map;

public:
    LaneEncapsulatingVariable(IVariable<geometry::Vec> const *parent, map::IMap const *map);

    structures::stl::STLStackArray<uint64_t> get_value() const override;
};

}
}
}
