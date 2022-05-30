#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_abstract.hpp>
#include <ori/simcars/map/weak_lane_array_interface.hpp>
#include <ori/simcars/map/weak_traffic_light_array_interface.hpp>

#include <functional>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class ALivingLane : public ALane<T_id>
{
    mutable std::weak_ptr<const ILane<T_id>> left_adjacent_lane;
    mutable std::weak_ptr<const ILane<T_id>> right_adjacent_lane;
    mutable std::shared_ptr<const IWeakLaneArray<T_id>> fore_lanes;
    mutable std::shared_ptr<const IWeakLaneArray<T_id>> aft_lanes;
    mutable std::shared_ptr<const IWeakTrafficLightArray<T_id>> traffic_lights;

    std::function<std::shared_ptr<const ILane<T_id>>(const std::weak_ptr<const ILane<T_id>>&)> get_shared_lane_func =
            [] (const std::weak_ptr<const ILane<T_id>>& weak_lane) { return weak_lane.lock(); };
    std::function<std::shared_ptr<const ITrafficLight<T_id>>(const std::weak_ptr<const ITrafficLight<T_id>>&)> get_shared_traffic_light_func =
            [] (const std::weak_ptr<const ITrafficLight<T_id>>& weak_traffic_light) { return weak_traffic_light.lock(); };

protected:
    void set_left_adjacent_lane(std::shared_ptr<const ILane<T_id>> left_adjacent_lane)
    {
        this->left_adjacent_lane = left_adjacent_lane;
    }
    void set_right_adjacent_lane(std::shared_ptr<const ILane<T_id>> right_adjacent_lane)
    {
        this->right_adjacent_lane = right_adjacent_lane;
    }
    void set_fore_lanes(std::shared_ptr<const IWeakLaneArray<T_id>> fore_lanes)
    {
        this->fore_lanes = fore_lanes;
    }
    void set_aft_lanes(std::shared_ptr<const IWeakLaneArray<T_id>> aft_lanes)
    {
        this->aft_lanes = aft_lanes;
    }
    void set_traffic_lights(std::shared_ptr<const IWeakTrafficLightArray<T_id>> traffic_lights)
    {
        this->traffic_lights = traffic_lights;
    }

public:
    ALivingLane(const T_id& id, std::shared_ptr<const IMap<T_id>> map) : ALane<T_id>(id, map) {}
    ~ALivingLane() override
    {
        std::shared_ptr<const ILane<T_id>> left_adjacent_lane = this->left_adjacent_lane.lock();
        if (left_adjacent_lane) left_adjacent_lane->banish();

        std::shared_ptr<const ILane<T_id>> right_adjacent_lane = this->right_adjacent_lane.lock();
        if (right_adjacent_lane) right_adjacent_lane->banish();
    }

    std::shared_ptr<const ILane<T_id>> get_true_self() const noexcept override
    {
        return std::shared_ptr<const ILane<T_id>>(this);
    }

    std::shared_ptr<const ILane<T_id>> get_left_adjacent_lane() const override
    {
        std::shared_ptr<const ILane<T_id>> left_adjacent_lane = this->left_adjacent_lane.lock();
        try
        {
            return left_adjacent_lane->get_self();
        }
        catch (typename ALane<T_id>::GhostObjectException)
        {
            left_adjacent_lane = left_adjacent_lane->get_true_self();
            this->left_adjacent_lane = left_adjacent_lane;
            return left_adjacent_lane;
        }
    }
    std::shared_ptr<const ILane<T_id>> get_right_adjacent_lane() const override
    {
        std::shared_ptr<const ILane<T_id>> right_adjacent_lane = this->right_adjacent_lane.lock();
        try
        {
            return right_adjacent_lane->get_self();
        }
        catch (typename ALane<T_id>::GhostObjectException)
        {
            right_adjacent_lane = right_adjacent_lane->get_true_self();
            this->right_adjacent_lane = right_adjacent_lane;
            return right_adjacent_lane;
        }
    }
    std::shared_ptr<const ILaneArray<T_id>> get_fore_lanes() const override
    {
        std::shared_ptr<const IWeakLaneArray<T_id>> weak_fore_lanes;
        try
        {
            weak_fore_lanes = fore_lanes->get_self();
        }
        catch (typename IWeakLaneArray<T_id>::GhostObjectException)
        {
            fore_lanes = fore_lanes->get_true_self();
            weak_fore_lanes = fore_lanes;
        }
        std::shared_ptr<structures::AArray<std::shared_ptr<const ILane<T_id>>>> fore_lanes(
                    new structures::stl::STLStackArray<std::shared_ptr<const ILane<T_id>>>(weak_fore_lanes->count()));
        fore_lanes->template map_from<std::weak_ptr<const ILane<T_id>>>(get_shared_lane_func, *weak_fore_lanes);
        return fore_lanes;
    }
    std::shared_ptr<const ILaneArray<T_id>> get_aft_lanes() const override
    {
        std::shared_ptr<const IWeakLaneArray<T_id>> weak_aft_lanes;
        try
        {
            weak_aft_lanes = aft_lanes->get_self();
        }
        catch (typename IWeakLaneArray<T_id>::GhostObjectException)
        {
            aft_lanes = aft_lanes->get_true_self();
            weak_aft_lanes = aft_lanes;
        }
        std::shared_ptr<structures::AArray<std::shared_ptr<const ILane<T_id>>>> aft_lanes(
                    new structures::stl::STLStackArray<std::shared_ptr<const ILane<T_id>>>(weak_aft_lanes->count()));
        aft_lanes->template map_from<std::weak_ptr<const ILane<T_id>>>(get_shared_lane_func, *weak_aft_lanes);
        return aft_lanes;
    }
    std::shared_ptr<const ITrafficLightArray<T_id>> get_traffic_lights() const override
    {
        std::shared_ptr<const IWeakTrafficLightArray<T_id>> weak_traffic_lights;
        try
        {
            weak_traffic_lights = traffic_lights->get_self();
        }
        catch (typename IWeakTrafficLightArray<T_id>::GhostObjectException)
        {
            traffic_lights = traffic_lights->get_true_self();
            weak_traffic_lights = traffic_lights;
        }
        std::shared_ptr<structures::AArray<std::shared_ptr<const ITrafficLight<T_id>>>> traffic_lights(
                    new structures::stl::STLStackArray<std::shared_ptr<const ITrafficLight<T_id>>>(weak_traffic_lights->count()));
        traffic_lights->template map_from<std::weak_ptr<const ITrafficLight<T_id>>>(get_shared_traffic_light_func, *weak_traffic_lights);
        return traffic_lights;
    }
};

}
}
}
