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
    map::IDrivingMap const *map;

public:
    LaneEncapsulatingVariable(IVariable<geometry::Vec> *parent, map::IDrivingMap const *map);

    bool get_value(structures::stl::STLStackArray<uint64_t> &val) const override;

    bool set_value(structures::stl::STLStackArray<uint64_t> const &val) override;
};

}
}
}
