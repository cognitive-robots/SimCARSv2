#pragma once

#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class LaneEncapsulationVariable : public ABinaryEndogenousVariable<bool, uint64_t, geometry::Vec>
{
    map::IMap const *map;

public:
    LaneEncapsulationVariable(IEndogenousVariable<uint64_t> *endogenous_parent,
                              IVariable<geometry::Vec> *other_parent, map::IMap const *map);

    bool get_value(bool &val) const override;

    bool set_value(bool const &val) override;
};

}
}
}
