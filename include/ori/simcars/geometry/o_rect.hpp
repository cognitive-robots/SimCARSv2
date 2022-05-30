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
    FP_DATA_TYPE orientation = NAN;

protected:
    std::shared_ptr<const TrigBuff> trig_buff;

    bool check_collision_virt(const Rect& rect) const override;
    void calc_bounds_virt() const override;

    bool align_and_check_bounds(const Vec& point) const;
    bool align_and_check_bounds(const ORect& o_rect) const;

    virtual bool check_collision_virt(const ORect &o_rect) const;

public:
    ORect();
    ORect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height, FP_DATA_TYPE orientation);
    ORect(const Rect& rect);
    ORect(const Rect& rect, FP_DATA_TYPE orientation);
    ORect(const ORect& o_rect);

    bool operator ==(const ORect& rect) const;
    bool check_collision(const ORect &rect) const;

    void rotate(FP_DATA_TYPE rotation);
    void rotate(FP_DATA_TYPE rotation, Vec rotation_centre);

    bool check_encapsulation(const Vec& point) const override;
};

}
}
}
