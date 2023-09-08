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
    FP_DATA_TYPE half_width, half_height, half_span;
    mutable FP_DATA_TYPE min_x, min_y, max_x, max_y;
    mutable Vec points[4];
    mutable bool calc_bounds_flag, calc_points_flag;

protected:
    void set_min_x(FP_DATA_TYPE min_x) const;
    void set_min_y(FP_DATA_TYPE min_y) const;
    void set_max_x(FP_DATA_TYPE max_x) const;
    void set_max_y(FP_DATA_TYPE max_y) const;
    void set_points(Vec const &point_1, Vec const &point_2, Vec const &point_3,
                    Vec const &point_4) const;
    void set_calc_bounds_flag() const;
    void set_calc_points_flag() const;
    void calc_bounds() const;
    void calc_points() const;
    bool check_bounds(Vec const &point) const;
    bool check_bounds(Rect const &rect) const;

    virtual bool check_collision_virt(Rect const &rect) const;
    virtual void calc_bounds_virt() const;
    virtual void calc_points_virt() const;

public:
    Rect();
    Rect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height);
    Rect(FP_DATA_TYPE min_x, FP_DATA_TYPE min_y, FP_DATA_TYPE max_x, FP_DATA_TYPE max_y);
    Rect(Vecs points);
    Rect(Rect const &rect);
    Rect(Rect const &rect_1, Rect const &rect_2);

    virtual ~Rect() = default;

    bool operator ==(Rect const &rect) const;
    Vec const& get_origin() const;
    FP_DATA_TYPE get_half_width() const;
    FP_DATA_TYPE get_half_height() const;
    FP_DATA_TYPE get_half_span() const;
    FP_DATA_TYPE get_width() const;
    FP_DATA_TYPE get_height() const;
    FP_DATA_TYPE get_span() const;
    FP_DATA_TYPE get_min_x() const;
    FP_DATA_TYPE get_min_y() const;
    FP_DATA_TYPE get_max_x() const;
    FP_DATA_TYPE get_max_y() const;
    Vec const& operator [](size_t idx) const;
    bool check_vicinity(Vec const &point) const;
    bool map_point(Vec const &point, Vec &mapped_point) const;
    bool check_vicinity(Rect const &rect) const;
    bool check_collision(Rect const &rect) const;
    VecPair calc_contact(Rect const &rect) const;

    void set_origin(Vec const &origin);
    void set_width(FP_DATA_TYPE width);
    void set_height(FP_DATA_TYPE height);
    void translate(Vec translation);

    virtual bool check_encapsulation(Vec const &point) const;
};

}
}
}
