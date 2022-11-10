
#include <ori/simcars/geometry/o_rect.hpp>

#include <cmath>

namespace ori
{
namespace simcars
{
namespace geometry
{

ORect::ORect() : orientation(0.0f) {}

ORect::ORect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height, FP_DATA_TYPE orientation)
    : Rect(origin, width, height), orientation(orientation), trig_buff(TrigBuff::get_instance()) {}

ORect::ORect(Rect const &rect) : ORect(rect, 0.0f) {}

ORect::ORect(Rect const &rect, FP_DATA_TYPE orientation)
    : Rect(rect), orientation(orientation), trig_buff(TrigBuff::get_instance()) {}

ORect::ORect(ORect const &o_rect)
    : Rect(o_rect), orientation(o_rect.orientation), trig_buff(o_rect.trig_buff) {}

bool ORect::check_collision_virt(Rect const &rect) const
{
    return this->check_bounds(rect) && this->align_and_check_bounds(rect);
}

void ORect::calc_bounds_virt() const
{
    FP_DATA_TYPE sin_o = trig_buff->get_sin(orientation);
    FP_DATA_TYPE cos_o = trig_buff->get_cos(orientation);
    FP_DATA_TYPE aligned_half_width = std::abs(get_half_width() * cos_o) + std::abs(get_half_height() * sin_o);
    FP_DATA_TYPE aligned_half_height = std::abs(get_half_height() * cos_o) + std::abs(get_half_width() * sin_o);
    set_min_x(get_origin().x() - aligned_half_width);
    set_min_y(get_origin().y() - aligned_half_height);
    set_max_x(get_origin().x() + aligned_half_width);
    set_max_y(get_origin().y() + aligned_half_height);
}

bool ORect::align_and_check_bounds(Vec const &point) const
{
    ORect aligned_this(this->get_origin(), this->get_width(), this->get_height(), 0.0f);
    Vec aligned_point = this->get_origin() + trig_buff->get_rot_mat(-this->orientation) * (point - this->get_origin());
    return aligned_this.check_bounds(aligned_point);
}

bool ORect::align_and_check_bounds(ORect const &o_rect) const
{
    ORect aligned_this(this->get_origin(), this->get_width(), this->get_height(), 0.0f);
    ORect aligned_o_rect = o_rect;
    aligned_o_rect.rotate(-this->orientation, this->get_origin());
    return aligned_this.check_bounds(aligned_o_rect);
}

bool ORect::check_collision_virt(ORect const &o_rect) const
{
    return this->check_bounds(o_rect) && this->align_and_check_bounds(o_rect);
}

bool ORect::operator ==(ORect const &o_rect) const
{
    return Rect::operator ==(o_rect) && this->orientation == o_rect.orientation;
}

bool ORect::check_collision(ORect const &o_rect) const
{
    return this->check_collision_virt(o_rect) && o_rect.check_collision_virt(*this);
}

void ORect::rotate(FP_DATA_TYPE rotation)
{
    orientation += rotation;
    set_calc_bounds_flag();
}

void ORect::rotate(FP_DATA_TYPE rotation, Vec rotation_centre)
{
    set_origin(rotation_centre + trig_buff->get_rot_mat(rotation) * (get_origin() - rotation_centre));
    rotate(rotation);
}

bool ORect::check_encapsulation(Vec const &point) const
{
    return check_bounds(point) && align_and_check_bounds(point);
}

}
}
}
