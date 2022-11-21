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
    mutable ILaneArray<T_id> const *fore_lanes;
    mutable ILaneArray<T_id> const *aft_lanes;
    mutable ITrafficLightArray<T_id> const *traffic_lights;

protected:
    void set_left_adjacent_lane(ILane<T_id> const *left_adjacent_lane)
    {
        this->left_adjacent_lane = left_adjacent_lane;
    }
    void set_right_adjacent_lane(ILane<T_id> const *right_adjacent_lane)
    {
        this->right_adjacent_lane = right_adjacent_lane;
    }
    void set_fore_lanes(ILaneArray<T_id> const *fore_lanes)
    {
        this->fore_lanes = fore_lanes;
    }
    void set_aft_lanes(ILaneArray<T_id> const *aft_lanes)
    {
        this->aft_lanes = aft_lanes;
    }
    void set_traffic_lights(ITrafficLightArray<T_id> const *traffic_lights)
    {
        this->traffic_lights = traffic_lights;
    }

public:
    ALivingLane(T_id const &id, IMap<T_id> const *map) :
        ALane<T_id>(id, map), left_adjacent_lane(nullptr), right_adjacent_lane(nullptr),
        straight_fore_lane(nullptr), fore_lanes(nullptr), aft_lanes(nullptr), traffic_lights(nullptr) {}
    ~ALivingLane() override
    {
        delete fore_lanes;
        delete aft_lanes;
        delete traffic_lights;
    }

    bool is_ghost() const override
    {
        return false;
    }
    ILane<T_id> const* get_true_self() const noexcept override
    {
        return this;
    }

    ILane<T_id> const* get_left_adjacent_lane() const override
    {
        if (left_adjacent_lane == nullptr)
        {
            return nullptr;
        }
        try
        {
            return left_adjacent_lane->get_self();
        }
        catch (typename ALane<T_id>::GhostObjectException)
        {
            left_adjacent_lane = left_adjacent_lane->get_true_self();
            return left_adjacent_lane;
        }
    }
    ILane<T_id> const* get_right_adjacent_lane() const override
    {
        if (right_adjacent_lane == nullptr)
        {
            return nullptr;
        }
        try
        {
            return right_adjacent_lane->get_self();
        }
        catch (typename ALane<T_id>::GhostObjectException)
        {
            right_adjacent_lane = right_adjacent_lane->get_true_self();
            return right_adjacent_lane;
        }
    }
    ILane<T_id> const* get_straight_fore_lane() const override
    {
        if (straight_fore_lane == nullptr)
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
                straight_fore_lane = min_abs_steer_fore_lane;
                return min_abs_steer_fore_lane;
            }
            else
            {
                return nullptr;
            }
        }
        else
        {
            return straight_fore_lane;
        }
    }
    ILaneArray<T_id> const* get_fore_lanes() const override
    {
        try
        {
            return fore_lanes->get_self();
        }
        catch (typename ILaneArray<T_id>::GhostObjectException)
        {
            fore_lanes = fore_lanes->get_true_self();
            return fore_lanes;
        }
    }
    ILaneArray<T_id> const* get_aft_lanes() const override
    {
        try
        {
            return aft_lanes->get_self();
        }
        catch (typename ILaneArray<T_id>::GhostObjectException)
        {
            aft_lanes = aft_lanes->get_true_self();
            return aft_lanes;
        }
    }
    ITrafficLightArray<T_id> const* get_traffic_lights() const override
    {
        try
        {
            return traffic_lights->get_self();
        }
        catch (typename ITrafficLightArray<T_id>::GhostObjectException)
        {
            traffic_lights = traffic_lights->get_true_self();
            return traffic_lights;
        }
    }
};

}
}
}
