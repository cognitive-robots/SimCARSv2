
#include <ori/simcars/map/driving_map_object_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

ADrivingMapObject::ADrivingMapObject(uint64_t id, IDrivingMap const *map) : id(id), map(map) {}

uint64_t ADrivingMapObject::get_id() const
{
    return id;
}

IDrivingMap const* ADrivingMapObject::get_map() const
{
    return map;
}

}
}
}
