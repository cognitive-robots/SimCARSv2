
#include <ori/simcars/geometry/rect.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace geometry
{

Rect::Rect() : Rect(0.0f, 0.0f, 0.0f, 0.0f) {}

Rect::Rect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height)
    : origin(origin), half_width(0.5f * width), half_height(0.5f * height),
      half_span(0.5f * std::sqrt(std::pow(width, 2.0f) + std::pow(height, 2.0f))),
      calc_bounds_flag(true), calc_points_flag(true) {}

Rect::Rect(FP_DATA_TYPE min_x, FP_DATA_TYPE min_y, FP_DATA_TYPE max_x, FP_DATA_TYPE max_y)
    : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y), calc_bounds_flag(false),
      calc_points_flag(true)
{
    assert(max_x >= min_x);
    assert(max_y >= min_y);

    origin << (min_x + max_x) / 2.0f, (min_y + max_y) / 2.0f;
    half_width = 0.5f * (max_x - min_x);
    half_height = 0.5f * (max_y - min_y);
    half_span = std::sqrt(std::pow(half_width, 2.0f) +
                          std::pow(half_height, 2.0f));
}

Rect::Rect(Vecs points)
    : Rect(points.row(0).minCoeff(), points.row(1).minCoeff(), points.row(0).maxCoeff(),
           points.row(1).maxCoeff()) {}

Rect::Rect(Rect const &rect)
    : origin(rect.origin), half_width(rect.half_width), half_height(rect.half_height),
      half_span(rect.half_span), min_x(rect.min_x), min_y(rect.min_y),
      max_x(rect.max_x), max_y(rect.max_y), calc_bounds_flag(rect.calc_bounds_flag),
      calc_points_flag(rect.calc_points_flag) {}

Rect::Rect(Rect const &rect_1, Rect const &rect_2)
    : Rect(std::min(rect_1.get_min_x(), rect_2.get_min_x()),
           std::min(rect_1.get_min_y(), rect_2.get_min_y()),
           std::max(rect_1.get_max_x(), rect_2.get_max_x()),
           std::max(rect_1.get_max_y(), rect_2.get_max_y())) {}

void Rect::set_min_x(FP_DATA_TYPE min_x) const
{
    this->min_x = min_x;
}

void Rect::set_min_y(FP_DATA_TYPE min_y) const
{
    this->min_y = min_y;
}

void Rect::set_max_x(FP_DATA_TYPE max_x) const
{
    this->max_x = max_x;
}

void Rect::set_max_y(FP_DATA_TYPE max_y) const
{
    this->max_y = max_y;
}

void Rect::set_points(Vec const &point_1, Vec const &point_2, Vec const &point_3,
                      Vec const &point_4) const
{
    points[0] = point_1;
    points[1] = point_2;
    points[2] = point_3;
    points[3] = point_4;
}

void Rect::set_calc_bounds_flag() const
{
    calc_bounds_flag = true;
}

void Rect::set_calc_points_flag() const
{
    calc_points_flag = true;
}

void Rect::calc_bounds() const
{
    calc_bounds_virt();
    calc_bounds_flag = false;
}

void Rect::calc_points() const
{
    calc_points_virt();
    calc_points_flag = false;
}

bool Rect::check_bounds(Vec const &point) const
{
    if (calc_bounds_flag) calc_bounds();
    return !(min_x > point.x() || max_x < point.x()
             || min_y > point.y() || max_y < point.y());
}

bool Rect::check_bounds(Rect const &rect) const
{
    if (this->calc_bounds_flag) this->calc_bounds();
    if (rect.calc_bounds_flag) rect.calc_bounds();
    return !(this->min_x > rect.max_x || this->max_x < rect.min_x
             || this->min_y > rect.max_y || this->max_y < rect.min_y);
}

bool Rect::check_collision_virt(Rect const &rect) const
{
    return true;
}

void Rect::calc_bounds_virt() const
{
    min_x = origin.x() - half_width;
    min_y = origin.y() - half_height;
    max_x = origin.x() + half_width;
    max_y = origin.y() + half_height;
}

void Rect::calc_points_virt() const
{
    points[0] = origin + Vec(half_width, half_height);
    points[1] = origin + Vec(-half_width, half_height);
    points[2] = origin + Vec(-half_width, -half_height);
    points[3] = origin + Vec(half_width, -half_height);
}

bool Rect::operator ==(Rect const &rect) const
{
    return this->origin == rect.origin && this->half_width == rect.half_width &&
            this->half_height == rect.half_height;
}

Vec const& Rect::get_origin() const
{
    return origin;
}

FP_DATA_TYPE Rect::get_half_width() const
{
    return half_width;
}

FP_DATA_TYPE Rect::get_half_height() const
{
    return half_height;
}

FP_DATA_TYPE Rect::get_half_span() const
{
    return half_span;
}

FP_DATA_TYPE Rect::get_width() const
{
    return 2.0f * half_width;
}

FP_DATA_TYPE Rect::get_height() const
{
    return 2.0f * half_height;
}

FP_DATA_TYPE Rect::get_span() const
{
    return 2.0f * half_span;
}

FP_DATA_TYPE Rect::get_min_x() const
{
    return min_x;
}

FP_DATA_TYPE Rect::get_min_y() const
{
    return min_y;
}

FP_DATA_TYPE Rect::get_max_x() const
{
    return max_x;
}

FP_DATA_TYPE Rect::get_max_y() const
{
    return max_y;
}

bool Rect::check_vicinity(Vec const &point) const
{
    return (point - origin).norm() <= half_span;
}

bool Rect::map_point(Vec const &point, Vec &mapped_point) const
{
    if (calc_points_flag) calc_points();

    for (size_t i = 0; i < 4; ++i)
    {
        Vec along_edge = points[i + 1 % 4] - points[i];
        Vec to_point = point - points[i];
        Vec along_edge_normalised = along_edge.normalized();
        FP_DATA_TYPE mapped_len_along_edge = to_point.dot(along_edge_normalised);

        Vec prospective_mapped_point;
        if (mapped_len_along_edge >= 0 && mapped_len_along_edge <= along_edge.norm())
        {
            prospective_mapped_point = points[i] + mapped_len_along_edge * along_edge_normalised;
        }
        else
        {
            prospective_mapped_point = points[i];
        }

        if (i == 0 || (point - prospective_mapped_point).norm() < (point - mapped_point).norm())
        {
            mapped_point = prospective_mapped_point;
        }
    }

    bool point_inside = (origin - point).norm() <= (origin - mapped_point).norm();

    return point_inside;
}

bool Rect::check_vicinity(Rect const &rect) const
{
    return (rect.origin - this->origin).norm() <= (this->half_span + rect.half_span);
}

bool Rect::check_collision(Rect const &rect) const
{
    return check_vicinity(rect) && check_bounds(rect) &&
            this->check_collision_virt(rect) && rect.check_collision_virt(*this);
}

VecPair Rect::calc_contact(Rect const &rect) const
{
    if (this->calc_points_flag) this->calc_points();
    if (rect.calc_points_flag) rect.calc_points();

    Vec best_point;
    Vec best_mapped_point;
    bool best_is_inside;

    size_t i;
    for (i = 0; i < 4; ++i)
    {
        Vec mapped_point;
        bool is_inside = rect.map_point(this->points[i], mapped_point);
        if (i == 0)
        {
            best_point = this->points[i];
            best_mapped_point = mapped_point;
            best_is_inside = is_inside;
        }
        else
        {
            if (best_is_inside)
            {
                if (is_inside &&
                        (this->points[i] - mapped_point).norm() >
                        (best_point - best_mapped_point).norm())
                {
                    best_point = this->points[i];
                    best_mapped_point = mapped_point;
                    best_is_inside = is_inside;
                }
            }
            else
            {
                if (is_inside ||
                        (mapped_point - this->points[i]).norm() <
                        (best_mapped_point - best_point).norm())
                {
                    best_point = this->points[i];
                    best_mapped_point = mapped_point;
                    best_is_inside = is_inside;
                }
            }
        }
    }
    for (i = 0; i < 4; ++i)
    {
        Vec mapped_point;
        bool is_inside = this->map_point(rect.points[i], mapped_point);
        if (best_is_inside)
        {
            if (is_inside &&
                    (this->points[i] - mapped_point).norm() >
                    (best_point - best_mapped_point).norm())
            {
                best_point = this->points[i];
                best_mapped_point = mapped_point;
                best_is_inside = is_inside;
            }
        }
        else
        {
            if (is_inside ||
                    (mapped_point - this->points[i]).norm() <
                    (best_mapped_point - best_point).norm())
            {
                best_point = this->points[i];
                best_mapped_point = mapped_point;
                best_is_inside = is_inside;
            }
        }
    }

    return VecPair(best_mapped_point, (best_point - best_mapped_point).normalized());
}

void Rect::set_origin(Vec const &origin)
{
    this->origin = origin;
    set_calc_bounds_flag();
    set_calc_points_flag();
}

void Rect::set_width(FP_DATA_TYPE width)
{
    half_width = 0.5f * width;
    half_span = std::sqrt(std::pow(half_width, 2.0f) +
                          std::pow(half_height, 2.0f));
    set_calc_bounds_flag();
    set_calc_points_flag();
}

void Rect::set_height(FP_DATA_TYPE height)
{
    half_height = 0.5f * height;
    half_span = std::sqrt(std::pow(half_width, 2.0f) +
                          std::pow(half_height, 2.0f));
    set_calc_bounds_flag();
    set_calc_points_flag();
}

void Rect::translate(Vec translation)
{
    origin += translation;
    set_calc_bounds_flag();
    set_calc_points_flag();
}

bool Rect::check_encapsulation(Vec const &point) const
{
    return check_vicinity(point) && check_bounds(point);
}

}
}
}
