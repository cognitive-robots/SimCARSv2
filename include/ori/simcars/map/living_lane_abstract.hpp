#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_abstract.hpp>

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
    mutable ILane<T_id> const *left_adjacent_lane;
    mutable ILane<T_id> const *right_adjacent_lane;
    mutable ILane<T_id> const *straight_fore_lane;
    mutable IWeakLaneArray<T_id> const *fore_lanes;
    mutable IWeakLaneArray<T_id> const *aft_lanes;
    mutable IWeakTrafficLightArray<T_id> const *traffic_lights;

protected:
    void set_left_adjacent_lane(ILane<T_id> const *left_adjacent_lane)
    {
        this->left_adjacent_lane = left_adjacent_lane;
    }
    void set_right_adjacent_lane(ILane<T_id> const *right_adjacent_lane)
    {
        this->right_adjacent_lane = right_adjacent_lane;
    }
    void set_fore_lanes(IWeakLaneArray<T_id> const *fore_lanes)
    {
        this->fore_lanes = fore_lanes;
    }
    void set_aft_lanes(IWeakLaneArray<T_id> const *aft_lanes)
    {
        this->aft_lanes = aft_lanes;
    }
    void set_traffic_lights(IWeakTrafficLightArray<T_id> const *traffic_lights)
    {
        this->traffic_lights = traffic_lights;
    }

public:
    ALivingLane(T_id const &id, IMap<T_id> const *map) : ALane<T_id>(id, map) {}
    ~ALivingLane() override
    {
        if (fore_lanes != nullptr)
        {
            delete fore_lanes;
        }

        if (aft_lanes != nullptr)
        {
            delete aft_lanes;
        }

        if (traffic_lights != nullptr)
        {
            delete traffic_lights;
        }
    }

    ILane<T_id> const* get_true_self() const noexcept override
    {
        return this->shared_from_this();
    }

    ILane<T_id> const* get_left_adjacent_lane() const override
    {
        if (this->left_adjacent_lane.expired())
        {
            return nullptr;
        }
        ILane<T_id> const *left_adjacent_lane = this->left_adjacent_lane.lock();
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
    ILane<T_id> const* get_right_adjacent_lane() const override
    {
        if (this->right_adjacent_lane.expired())
        {
            return nullptr;
        }
        ILane<T_id> const *right_adjacent_lane = this->right_adjacent_lane.lock();
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
    ILane<T_id> const* get_straight_fore_lane() const override
    {
        if (this->straight_fore_lane.expired())
        {
            ILaneArray<T_id> const *fore_lanes = this->get_fore_lanes();
            if (fore_lanes->count() > 0)
            {
                ILane<T_id> const *min_abs_steer_fore_lane = (*fore_lanes)[0];
                for (size_t i = 1; i < fore_lanes->count(); ++i)
                {
                    if (std::abs((*fore_lanes)[i]->get_mean_steer()) < std::abs(min_abs_steer_fore_lane->get_mean_steer()))
                    {
                        min_abs_steer_fore_lane = (*fore_lanes)[i];
                    }
                }
                this->straight_fore_lane = min_abs_steer_fore_lane;
                return min_abs_steer_fore_lane;
            }
            else
            {
                return nullptr;
            }
        }
        else
        {
            return this->straight_fore_lane.lock();
        }
    }
    ILaneArray<T_id> const* get_fore_lanes() const override
    {
        IWeakLaneArray<T_id> const *fore_lanes;
        try
        {
            fore_lanes = this->fore_lanes->get_self();
        }
        catch (typename IWeakLaneArray<T_id>::GhostObjectException)
        {
            this->fore_lanes = this->fore_lanes->get_true_self();
            fore_lanes = this->fore_lanes;
        }
        structures::IArray<ILane<T_id> const*> *fore_lanes =
                new structures::stl::STLStackArray<ILane<T_id> const*>(fore_lanes->count());
        map_array(*fore_lanes, *fore_lanes, get_shared_lane_func);
        return fore_lanes;
    }
    ILaneArray<T_id> const* get_aft_lanes() const override
    {
        IWeakLaneArray<T_id> const *aft_lanes;
        try
        {
            aft_lanes = this->aft_lanes->get_self();
        }
        catch (typename IWeakLaneArray<T_id>::GhostObjectException)
        {
            this->aft_lanes = this->aft_lanes->get_true_self();
            aft_lanes = this->aft_lanes;
        }
        structures::IArray<ILane<T_id> const*> *aft_lanes =
                new structures::stl::STLStackArray<ILane<T_id> const*>(aft_lanes->count());
        map_array(*aft_lanes, *aft_lanes, get_shared_lane_func);
        return aft_lanes;
    }
    ITrafficLightArray<T_id> const* get_traffic_lights() const override
    {
        IWeakTrafficLightArray<T_id> const *traffic_lights;
        try
        {
            traffic_lights = this->traffic_lights->get_self();
        }
        catch (typename IWeakTrafficLightArray<T_id>::GhostObjectException)
        {
            this->traffic_lights = this->traffic_lights->get_true_self();
            traffic_lights = this->traffic_lights;
        }
        structures::IArray<ITrafficLight<T_id> const*> *traffic_lights =
                new structures::stl::STLStackArray<ITrafficLight<T_id> const*>(traffic_lights->count());
        map_array(*traffic_lights, *traffic_lights, get_shared_traffic_light_func);
        return traffic_lights;
    }
};

}
}
}
