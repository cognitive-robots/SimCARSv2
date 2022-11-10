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
    Tri(Vec const &point_1, Vec const &point_2, Vec const &point_3);
    Tri();
    virtual ~Tri() = default;

    bool operator ==(Tri const &tri) const;
    Vec const& operator [](size_t idx) const;
    bool check_encapsulation(Vec const &point) const;

    Vec& operator [](size_t idx);
};

}
}
}
