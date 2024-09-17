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

class NodeAdjacentVariable : public AUnaryEndogenousVariable<
        structures::stl::STLStackArray<uint64_t>, uint64_t>
{
    map::IPedMap const *map;

public:
    NodeAdjacentVariable(IVariable<uint64_t> *parent, map::IPedMap const *map);

    bool get_value(structures::stl::STLStackArray<uint64_t> &val) const override;

    bool set_value(structures::stl::STLStackArray<uint64_t> const &val) override;
};

}
}
}
