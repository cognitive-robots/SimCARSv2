
#include <ori/simcars/causal/variable_types/endogenous/node_adjacent.hpp>

#include <ori/simcars/structures/stl/stl_set.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

NodeAdjacentVariable::NodeAdjacentVariable(IVariable<uint64_t> *parent,
                                           map::IPedMap const *map) :
    AUnaryEndogenousVariable(parent), map(map)
{
    assert(map != nullptr);
}

bool NodeAdjacentVariable::get_value(structures::stl::STLStackArray<uint64_t> &val) const
{
    uint64_t node_id;
    if (get_parent()->get_value(node_id))
    {
        map::INode const *node = map->get_node(node_id);

        if (node == nullptr)
        {
            val.clear();
        }
        else
        {
            map_array<map::INode const*, uint64_t>(*(node->get_adjacent()), val,
                                                   [](map::INode const *node) { return node->get_id(); });
        }

        // NOTE: Despite the variable being named adjacent, it includes the parent node id itself
        val.push_back(node->get_id());

        return true;
    }
    else
    {
        return false;
    }
}

bool NodeAdjacentVariable::set_value(structures::stl::STLStackArray<uint64_t> const &val)
{
    uint64_t node_id;
    if (get_parent()->get_value(node_id))
    {
        map::INode const *node = map->get_node(node_id);

        if (node == nullptr)
        {
            return val.count() == 0;
        }
        else
        {
            if (!val.contains(node_id))
            {
                return false;
            }

            structures::IArray<map::INode const*> const *adjacent_nodes = node->get_adjacent();

            if (val.count() != adjacent_nodes->count() + 1)
            {
                return false;
            }

            for (size_t i = 0; i < adjacent_nodes->count(); ++i)
            {
                if (!val.contains((*adjacent_nodes)[i]->get_id()))
                {
                    return false;
                }
            }

            return true;
        }
    }
    return true;
}

}
}
}
