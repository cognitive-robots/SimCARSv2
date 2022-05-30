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
    std::shared_ptr<structures::IStackArray<geometry::Tri>> tris;
    size_t point_count;
    geometry::Rect bounding_box;
    AccessRestriction access_restriction;

public:
    LyftLane(const std::string& id, std::shared_ptr<const IMap<std::string>> map, const rapidjson::Value::ConstObject& json_lane_data);

    const geometry::Vecs& get_left_boundary() const override;
    const geometry::Vecs& get_right_boundary() const override;
    std::shared_ptr<const structures::IArray<geometry::Tri>> get_tris() const override;
    bool check_encapsulation(const geometry::Vec& point) const override;
    const geometry::Vec& get_centroid() const override;
    size_t get_point_count() const override;
    const geometry::Rect& get_bounding_box() const override;
    LyftLane::AccessRestriction get_access_restriction() const override;
};


}
}
}
}
