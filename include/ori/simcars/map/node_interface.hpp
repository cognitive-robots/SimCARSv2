#pragma once

#include <ori/simcars/map/ped_map_object_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class INode : public virtual IPedMapObject
{
public:
    virtual geometry::Vec const& get_centroid() const = 0;
    virtual structures::IArray<INode const*> const* get_adjacent() const = 0;
};

}
}
}
