#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/rect.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/declarations.hpp>

#include <exception>

namespace ori
{
namespace simcars
{
namespace map
{

class IMap
{
public:
    virtual ~IMap() = default;

    virtual geometry::Vec get_map_centre() const = 0;
    virtual geometry::Rect get_bounding_box() const = 0;
    virtual FP_DATA_TYPE get_max_dim_size() const = 0;
};

}
}
}
