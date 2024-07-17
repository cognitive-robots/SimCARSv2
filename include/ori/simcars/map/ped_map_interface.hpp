#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_interface.hpp>

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
    virtual sf::Sprite* get_sprite() const = 0;
};

}
}
}
