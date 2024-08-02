
#include <ori/simcars/causal/variable_types/endogenous/node_closest.hpp>

#include <ori/simcars/structures/stl/stl_set.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

NodeClosestVariable::NodeClosestVariable(IVariable<geometry::Vec> *parent,
                                         map::IPedMap const *map) :
    AUnaryEndogenousVariable(parent), map(map)
{
    assert(map != nullptr);
}

bool NodeClosestVariable::get_value(uint64_t &val) const
{
    geometry::Vec pos;
    if (get_parent()->get_value(pos))
    {
        map::INode const *node = map->get_node(pos);

        if (node == nullptr)
        {
            val = 0;
        }
        else
        {
            val = node->get_id();
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool NodeClosestVariable::set_value(uint64_t const &val)
{
    // NOTE: Might be interpreted in other ways, e.g. could set the position to the node centroid
    geometry::Vec pos;
    if (get_parent()->get_value(pos))
    {
        return map->get_node(val) == map->get_node(pos);
    }
    else
    {
        return true;
    }
}

}
}
}
