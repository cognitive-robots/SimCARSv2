
#include <ori/simcars/map/thor_magni/thor_magni_node.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

ThorMagniNode::ThorMagniNode(uint64_t id, IPedMap const *map, geometry::Vec centroid) : id(id),
    map(map), centroid(centroid) {}

uint64_t ThorMagniNode::get_id() const
{
    return id;
}

IPedMap const* ThorMagniNode::get_map() const
{
    return map;
}

geometry::Vec const& ThorMagniNode::get_centroid() const
{
    return centroid;
}

structures::IArray<INode const*> const* ThorMagniNode::get_adjacent() const
{
    return adjacent.get_array();
}

bool ThorMagniNode::add_adjacent(INode const *node)
{
    if (adjacent.contains(node))
    {
        return false;
    }
    else
    {
        adjacent.insert(node);
        return true;
    }
}

bool ThorMagniNode::remove_adjacent(INode const *node)
{
    if (adjacent.contains(node))
    {
        adjacent.erase(node);
        return true;
    }
    else
    {
        return false;
    }
}

}
}
}
