#pragma once

#include <ori/simcars/map/node_interface.hpp>

#include <ori/simcars/structures/stl/stl_set.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class ThorMagniNode : public virtual INode
{
    uint64_t const id;
    IPedMap const* const map;

    geometry::Vec centroid;
    structures::stl::STLSet<INode const*> adjacent;

public:
    ThorMagniNode(uint64_t id, IPedMap const *map, geometry::Vec centroid);

    uint64_t get_id() const override;
    IPedMap const* get_map() const override;

    geometry::Vec const& get_centroid() const override;
    structures::IArray<INode const*> const* get_adjacent() const override;

    bool add_adjacent(INode const *node);
    bool remove_adjacent(INode const *node);
};

}
}
}
