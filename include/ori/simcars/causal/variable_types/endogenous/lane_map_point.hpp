#pragma once

#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class LaneMapPointVariable :
        public ABinaryEndogenousVariable<geometry::Vec, uint64_t, geometry::Vec>
{
    map::IMap const *map;

public:
    LaneMapPointVariable(IEndogenousVariable<uint64_t> const *endogenous_parent,
                                    IVariable<geometry::Vec> const *other_parent,
                                    map::IMap const *map);

    geometry::Vec get_value() const override;
};

}
}
}
