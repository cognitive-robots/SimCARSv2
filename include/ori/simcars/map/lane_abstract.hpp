#pragma once

#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/map/map_object_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class ALane : public virtual ILane, public virtual AMapObject
{
    uint64_t left_adjacent_lane_id, right_adjacent_lane_id;
    mutable ILane const *left_adjacent_lane, *right_adjacent_lane, *straightest_fore_lane;

    LaneBranch fore_lane_branch, aft_lane_branch;

public:
    ALane(uint64_t id, uint64_t left_adjacent_lane_id, uint64_t right_adjacent_lane_id,
          structures::IArray<uint64_t> *fore_lane_ids, structures::IArray<uint64_t> *aft_lane_ids,
          IMap const *map);

    Type get_type() const override;

    ILane const* get_left_adjacent_lane() const override;
    ILane const* get_right_adjacent_lane() const override;
    LaneBranch const* get_fore_lane_branch() const override;
    LaneBranch const* get_aft_lane_branch() const override;
};

}
}
}
