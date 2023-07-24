
#include <ori/simcars/map/map_object_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

AMapObject::AMapObject(uint64_t id, IMap const *map) : id(id), map(map) {}

uint64_t AMapObject::get_id() const
{
    return id;
}

IMap const* AMapObject::get_map() const
{
    return map;
}

}
}
}
