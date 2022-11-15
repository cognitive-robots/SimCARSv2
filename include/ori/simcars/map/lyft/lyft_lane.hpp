#pragma once

#include <ori/simcars/structures/stack_array_interface.hpp>
#include <ori/simcars/map/living_lane_abstract.hpp>
#include <ori/simcars/map/lyft/lyft_declarations.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <rapidjson/document.h>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

class LyftLane : public ALivingLane<std::string>
{
    geometry::Vecs left_boundary, right_boundary;
    geometry::Vec centroid;
    structures::IStackArray<geometry::Tri> *tris;
    size_t point_count;
    FP_DATA_TYPE mean_steer;
    geometry::Rect bounding_box;
    AccessRestriction access_restriction;

public:
    LyftLane(std::string const &id, IMap<std::string> const *map, rapidjson::Value::ConstObject const &json_lane_data);
    ~LyftLane() override;

    geometry::Vecs const& get_left_boundary() const override;
    geometry::Vecs const& get_right_boundary() const override;
    structures::IArray<geometry::Tri> const* get_tris() const override;
    bool check_encapsulation(geometry::Vec const &point) const override;
    geometry::Vec const& get_centroid() const override;
    size_t get_point_count() const override;
    geometry::Rect const& get_bounding_box() const override;
    FP_DATA_TYPE get_mean_steer() const override;
    ILane::AccessRestriction get_access_restriction() const override;
};


}
}
}
}
