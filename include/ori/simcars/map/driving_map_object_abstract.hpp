#pragma once

#include <ori/simcars/map/driving_map_object_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class ADrivingMapObject : public virtual IDrivingMapObject
{
    uint64_t const id;
    IDrivingMap const* const map;

public:
    ADrivingMapObject(uint64_t id, IDrivingMap const *map);

    uint64_t get_id() const override;
    IDrivingMap const* get_map() const override;
};

}
}
}
