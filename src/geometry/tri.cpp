
#include <ori/simcars/geometry/tri.hpp>

namespace ori
{
namespace simcars
{
namespace geometry
{

Tri::Tri(const Vec& point_1, const Vec& point_2, const Vec& point_3) : points{point_1, point_2, point_3} {}

Tri::Tri() : Tri(geometry::Vec(0, 0), geometry::Vec(0,0), geometry::Vec(0,0)) {}

bool Tri::operator ==(const Tri& tri) const
{
    return this->points[0] == tri.points[0]
            && this->points[1] == tri.points[1]
            && this->points[2] == tri.points[2];
}

const Vec& Tri::operator [](size_t idx) const
{
    return points[idx];
}

bool Tri::check_encapsulation(const Vec &point) const
{
    FP_DATA_TYPE dot_products[3];
    size_t i, j;
    for (i = 0; i < 3; ++i)
    {
        j = (i + 1) % 3;
        dot_products[i] = (point.x() - points[i].x()) * (points[i].y() - points[j].y())
                + (point.y() - points[i].y()) * (points[j].x() - points[i].x());
    }
    return (dot_products[0] >= 0 && dot_products[1] >= 0 && dot_products[2] >= 0)
            || (dot_products[0] <= 0 && dot_products[1] <= 0 && dot_products[2] <= 0);
}

Vec& Tri::operator [](size_t idx)
{
    return points[idx];
}

}
}
}
