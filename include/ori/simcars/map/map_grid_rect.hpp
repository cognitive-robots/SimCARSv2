#pragma once

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/grid_rect.hpp>
#include <ori/simcars/map/map_object.hpp>
#include <ori/simcars/map/living_lane_stack_array.hpp>
#include <ori/simcars/map/living_traffic_light_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class MapGridRect : public geometry::GridRect<MapGridRect<T_id>>
{
    structures::ISet<ILane<T_id> const*> *lanes;
    structures::ISet<ITrafficLight<T_id> const*> *traffic_lights;

public:
    MapGridRect(geometry::Vec origin, FP_DATA_TYPE size)
        : geometry::GridRect<MapGridRect<T_id>>(origin, size, size), lanes(new structures::stl::STLSet<ILane<T_id> const*>()),
          traffic_lights(new structures::stl::STLSet<ITrafficLight<T_id> const*>()) {}
    MapGridRect() : MapGridRect(geometry::Vec(0, 0), 0) {}

    ILaneArray<T_id> const* get_encapsulating_lanes(geometry::Vec point) const
    {
        structures::IArray<ILane<T_id> const*> const *lanes = this->lanes->get_array();
        LivingLaneStackArray<std::string> *encapsulating_lanes =
                new LivingLaneStackArray<std::string>;
        size_t i;
        for (i = 0; i < lanes->count(); ++i)
        {
            ILane<T_id> const *lane = (*lanes)[i];
            if (lane->check_encapsulation(point))
            {
                encapsulating_lanes->push_back(lane);
            }
        }
        return encapsulating_lanes;
    }
    structures::ISet<ILane<T_id> const*> const* get_lanes() const
    {
        return lanes;
    }
    structures::ISet<ITrafficLight<T_id> const*> const* get_traffic_lights() const
    {
        return traffic_lights;
    }

    void insert_lane(ILane<T_id> const *lane)
    {
        lanes->insert(lane);
    }

    void insert_traffic_light(ITrafficLight<T_id> const *traffic_light)
    {
        traffic_lights->insert(traffic_light);
    }

    void erase_lane(ILane<T_id> const *lane)
    {
        lanes->erase(lane);
    }

    void erase_traffic_light(ITrafficLight<T_id> const *traffic_light)
    {
        traffic_lights->erase(traffic_light);
    }
};

}
}
}
