#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/node_interface.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class NodeClosestVariable : public AUnaryEndogenousVariable<uint64_t, geometry::Vec>
{
    map::IPedMap const *map;

public:
    NodeClosestVariable(IVariable<geometry::Vec> *parent, map::IPedMap const *map);

    bool get_value(uint64_t &val) const override;

    bool set_value(uint64_t const &val) override;
};

}
}
}
