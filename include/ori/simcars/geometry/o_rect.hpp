#pragma once

#include <ori/simcars/geometry/rect.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>

namespace ori
{
namespace simcars
{
namespace geometry
{

class ORect : public Rect
{
    TrigBuff const* trig_buff;

    FP_DATA_TYPE orientation = NAN;

protected:
    bool check_collision_virt(Rect const &rect) const override;
    void calc_bounds_virt() const override;
    void calc_points_virt() const override;

    bool align_and_check_bounds(Vec const &point) const;
    bool align_and_check_bounds(ORect const &o_rect) const;

    virtual bool check_collision_virt(ORect const &o_rect) const;

public:
    ORect();
    ORect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height, FP_DATA_TYPE orientation);
    ORect(Rect const &rect);
    ORect(Rect const &rect, FP_DATA_TYPE orientation);
    ORect(ORect const &o_rect);

    bool operator ==(ORect const &o_rect) const;
    bool check_collision(ORect const &o_rect) const;

    void rotate(FP_DATA_TYPE rotation);
    void rotate(FP_DATA_TYPE rotation, Vec rotation_centre);

    bool check_encapsulation(Vec const &point) const override;
};

}
}
}
