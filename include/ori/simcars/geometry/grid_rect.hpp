#pragma once

#include <ori/simcars/geometry/rect.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace geometry
{

template <class T_grid_rect>
class GridRect : public Rect
{
    T_grid_rect const *above, *below, *left, *right;

public:
    GridRect() : Rect() {}
    GridRect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height) : Rect(origin, width, height) {}
    GridRect(FP_DATA_TYPE min_x, FP_DATA_TYPE min_y, FP_DATA_TYPE max_x, FP_DATA_TYPE max_y)
        : Rect(min_x, min_y, max_x, max_y) {}
    GridRect(Vecs points)
        : Rect(points) {}
    GridRect(GridRect<T_grid_rect> const &grid_rect)
        : Rect(grid_rect), above(grid_rect.above), below(grid_rect.below), left(grid_rect.left), right(grid_rect.right) {}
    ~GridRect() override
    {
        static_assert(std::is_base_of<GridRect<T_grid_rect>, T_grid_rect>::value, "T_grid_rect is not derived from GridRect");
    }

    T_grid_rect const* get_above() const
    {
        return above;
    }
    T_grid_rect const* get_below() const
    {
        return below;
    }
    T_grid_rect const* get_left() const
    {
        return left;
    }
    T_grid_rect const* get_right() const
    {
        return right;
    }

    void set_above(T_grid_rect const *above)
    {
        assert(this->get_min_x() == above->get_min_x());
        assert(this->get_max_x() == above->get_max_x());
        assert(this->get_max_y() == above->get_min_y());

        this->above = above;
    }
    void set_below(T_grid_rect const *below)
    {
        assert(this->get_min_x() == above->get_min_x());
        assert(this->get_max_x() == above->get_max_x());
        assert(this->get_min_y() == above->get_max_y());

        this->below = below;
    }
    void set_left(T_grid_rect const *left)
    {
        assert(this->get_min_y() == above->get_min_y());
        assert(this->get_max_y() == above->get_max_y());
        assert(this->get_min_x() == above->get_max_x());

        this->left = left;
    }
    void set_right(T_grid_rect const *right)
    {
        assert(this->get_min_y() == above->get_min_y());
        assert(this->get_max_y() == above->get_max_y());
        assert(this->get_max_x() == above->get_min_x());

        this->right = right;
    }
};

}
}
}
