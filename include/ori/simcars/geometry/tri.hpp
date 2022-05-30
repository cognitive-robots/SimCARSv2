#pragma once

#include <ori/simcars/geometry/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace geometry
{

class Tri
{
    Vec points[3];

public:
    Tri(const Vec& point_1, const Vec& point_2, const Vec& point_3);
    Tri();
    virtual ~Tri() = default;

    bool operator ==(const Tri& tri) const;
    const Vec& operator [](size_t idx) const;
    bool check_encapsulation(const Vec& point) const;

    Vec& operator [](size_t idx);
};

}
}
}
