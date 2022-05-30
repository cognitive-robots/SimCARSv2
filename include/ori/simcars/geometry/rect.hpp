#pragma once

#include <ori/simcars/geometry/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace geometry
{

class Rect
{
    Vec origin;
    FP_DATA_TYPE half_width, half_height;
    mutable FP_DATA_TYPE min_x, min_y, max_x, max_y;
    mutable bool calc_bounds_flag;

protected:
    FP_DATA_TYPE get_half_width() const;
    FP_DATA_TYPE get_half_height() const;
    void set_min_x(FP_DATA_TYPE min_x) const;
    void set_min_y(FP_DATA_TYPE min_y) const;
    void set_max_x(FP_DATA_TYPE max_x) const;
    void set_max_y(FP_DATA_TYPE max_y) const;
    void set_calc_bounds_flag() const;
    void calc_bounds() const;
    bool check_bounds(const Vec& point) const;
    bool check_bounds(const Rect& rect) const;

    virtual bool check_collision_virt(const Rect& rect) const;
    virtual void calc_bounds_virt() const;

public:
    Rect();
    Rect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height);
    Rect(FP_DATA_TYPE min_x, FP_DATA_TYPE min_y, FP_DATA_TYPE max_x, FP_DATA_TYPE max_y);
    Rect(Vecs points);
    Rect(const Rect& rect);
    Rect(const Rect& rect_1, const Rect& rect_2);
    virtual ~Rect() = default;

    bool operator ==(const Rect& rect) const;
    const Vec& get_origin() const;
    FP_DATA_TYPE get_width() const;
    FP_DATA_TYPE get_height() const;
    FP_DATA_TYPE get_min_x() const;
    FP_DATA_TYPE get_min_y() const;
    FP_DATA_TYPE get_max_x() const;
    FP_DATA_TYPE get_max_y() const;
    bool check_collision(const Rect& rect) const;

    void set_origin(const Vec& origin);
    void set_width(FP_DATA_TYPE width);
    void set_height(FP_DATA_TYPE height);
    void translate(Vec translation);

    virtual bool check_encapsulation(const Vec& point) const;
};

}
}
}
