
#include <ori/simcars/causal/variable_types/endogenous/node_centroid.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

NodeCentroidVariable::NodeCentroidVariable(IVariable<uint64_t> *parent, map::IPedMap const *map) :
    AUnaryEndogenousVariable(parent), map(map) {}

bool NodeCentroidVariable::get_value(geometry::Vec &val) const
{
    uint64_t node_id;
    if (get_parent()->get_value(node_id))
    {
        map::INode const *node = map->get_node(node_id);
        if (node != nullptr)
        {
            val = node->get_centroid();
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool NodeCentroidVariable::set_value(geometry::Vec const &val)
{
    map::INode const *node = map->get_node(val);
    if(node != nullptr)
    {
        return get_parent()->set_value(node->get_id());
    }
    else
    {
        return false;
    }
}

}
}
}
