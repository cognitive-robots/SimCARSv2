
#include <ori/simcars/map/ghost_lane.hpp>
#include <ori/simcars/map/ghost_lane_array.hpp>
#include <ori/simcars/map/ghost_traffic_light_array.hpp>
#include <ori/simcars/map/plg/plg_lane.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace plg
{

PLGLane::PLGLane(uint8_t id, IMap<uint8_t> const *map, geometry::Vecs *vertices) :
    ALivingLane(id, map), access_restriction(PLGLane::AccessRestriction::NO_RESTRICTION)
{

}

PLGLane::~PLGLane()
{
    delete tris;
}

geometry::Vecs const& PLGLane::get_left_boundary() const
{
    return left_boundary;
}

geometry::Vecs const& PLGLane::get_right_boundary() const
{
    return right_boundary;
}

structures::IArray<geometry::Tri> const* PLGLane::get_tris() const
{
    return tris;
}

bool PLGLane::check_encapsulation(geometry::Vec const &point) const
{
    if (bounding_box.check_encapsulation(point))
    {
        size_t i;
        for (i = 0; i < tris->count(); ++i)
        {
            if ((*tris)[i].check_encapsulation(point))
            {
                return true;
            }
        }
    }
    return false;
}

geometry::Vec const& PLGLane::get_centroid() const
{
    return centroid;
}

size_t PLGLane::get_point_count() const
{
    return point_count;
}

geometry::Rect const& PLGLane::get_bounding_box() const
{
    return bounding_box;
}

FP_DATA_TYPE PLGLane::get_mean_steer() const
{
    return mean_steer;
}

PLGLane::AccessRestriction PLGLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
