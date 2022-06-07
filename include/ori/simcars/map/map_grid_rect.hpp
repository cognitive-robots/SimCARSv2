#pragma once

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/grid_rect.hpp>
#include <ori/simcars/map/map_object.hpp>
#include <ori/simcars/map/weak_living_lane_stack_array.hpp>
#include <ori/simcars/map/weak_living_traffic_light_stack_array.hpp>

#include <memory>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class MapGridRect : public geometry::GridRect<MapGridRect<T_id>>
{
    std::shared_ptr<structures::ISet<std::shared_ptr<const ILane<T_id>>>> lanes;
    std::shared_ptr<structures::ISet<std::shared_ptr<const ITrafficLight<T_id>>>> traffic_lights;

public:
    MapGridRect(geometry::Vec origin, FP_DATA_TYPE size)
        : geometry::GridRect<MapGridRect<T_id>>(origin, size, size), lanes(new structures::stl::STLSet<std::shared_ptr<const ILane<T_id>>>()),
          traffic_lights(new structures::stl::STLSet<std::shared_ptr<const ITrafficLight<T_id>>>()) {}
    MapGridRect() : MapGridRect(geometry::Vec(0, 0), 0) {}

    std::shared_ptr<const ILaneArray<T_id>> get_encapsulating_lanes(geometry::Vec point) const
    {
        std::shared_ptr<const ILaneArray<T_id>> lanes = this->lanes->get_array();
        std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const ILane<std::string>>>> encapsulating_lanes(
                    new structures::stl::STLStackArray<std::shared_ptr<const ILane<std::string>>>());
        size_t i;
        for (i = 0; i < lanes->count(); ++i)
        {
            std::shared_ptr<const ILane<T_id>> lane = (*lanes)[i];
            if (lane->check_encapsulation(point))
            {
                encapsulating_lanes->push_back(lane);
            }
        }
        return encapsulating_lanes;
    }
    std::shared_ptr<const structures::ISet<std::shared_ptr<const ILane<T_id>>>> get_lanes() const
    {
        return lanes;
    }
    std::shared_ptr<const structures::ISet<std::shared_ptr<const ITrafficLight<T_id>>>> get_traffic_lights() const
    {
        return traffic_lights;
    }

    void insert_lane(std::shared_ptr<const ILane<T_id>> lane)
    {
        lanes->insert(lane);
    }

    void insert_traffic_light(std::shared_ptr<const ITrafficLight<T_id>> traffic_light)
    {
        traffic_lights->insert(traffic_light);
    }

    void erase_lane(std::shared_ptr<const ILane<T_id>> lane)
    {
        lanes->erase(lane);
    }

    void erase_traffic_light(std::shared_ptr<const ITrafficLight<T_id>> traffic_light)
    {
        traffic_lights->erase(traffic_light);
    }
};

}
}
}
