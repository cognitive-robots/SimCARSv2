#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/map/node_interface.hpp>

#include <SFML/Graphics.hpp>

#include <exception>

namespace ori
{
namespace simcars
{
namespace map
{

class IPedMap : public virtual IMap
{
public:
    virtual sf::Texture const& get_texture() const = 0;
    virtual FP_DATA_TYPE get_texture_scale() const = 0;
    virtual geometry::Vec const& get_texture_offset() const = 0;

    virtual structures::IArray<INode const*>* get_nodes() const = 0;
    virtual structures::IArray<INode const*>* get_goal_nodes() const = 0;
    virtual structures::IArray<geometry::VecPair>* get_edges() const = 0;
};

}
}
}
