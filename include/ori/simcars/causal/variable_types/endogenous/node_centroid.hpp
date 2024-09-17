#pragma once

#include <ori/simcars/map/node_interface.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class NodeCentroidVariable : public AUnaryEndogenousVariable<geometry::Vec, uint64_t>
{
    map::IPedMap const *map;

public:
    NodeCentroidVariable(IVariable<uint64_t> *parent, map::IPedMap const *map);

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}
