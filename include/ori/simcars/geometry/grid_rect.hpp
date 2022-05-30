#pragma once

#include <ori/simcars/geometry/rect.hpp>

#include <memory>
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
    std::weak_ptr<T_grid_rect> above, below, left, right;

public:
    GridRect() : Rect() {}
    GridRect(Vec origin, FP_DATA_TYPE width, FP_DATA_TYPE height) : Rect(origin, width, height) {}
    GridRect(FP_DATA_TYPE min_x, FP_DATA_TYPE min_y, FP_DATA_TYPE max_x, FP_DATA_TYPE max_y)
        : Rect(min_x, min_y, max_x, max_y) {}
    GridRect(Vecs points)
        : Rect(points) {}
    GridRect(const GridRect<T_grid_rect>& grid_rect)
        : Rect(grid_rect), above(grid_rect.above), below(grid_rect.below), left(grid_rect.left), right(grid_rect.right) {}
    ~GridRect() override
    {
        static_assert(std::is_base_of<GridRect<T_grid_rect>, T_grid_rect>::value, "T_grid_rect is not derived from GridRect");
    }

    std::shared_ptr<const T_grid_rect> get_above() const
    {
        return above.lock();
    }
    std::shared_ptr<const T_grid_rect> get_below() const
    {
        return below.lock();
    }
    std::shared_ptr<const T_grid_rect> get_left() const
    {
        return left.lock();
    }
    std::shared_ptr<const T_grid_rect> get_right() const
    {
        return right.lock();
    }

    void set_above(std::shared_ptr<const T_grid_rect> above)
    {
        assert(this->get_min_x() == above->get_min_x());
        assert(this->get_max_x() == above->get_max_x());
        assert(this->get_max_y() == above->get_min_y());

        this->above = above;
    }
    void set_below(std::shared_ptr<const T_grid_rect> below)
    {
        assert(this->get_min_x() == above->get_min_x());
        assert(this->get_max_x() == above->get_max_x());
        assert(this->get_min_y() == above->get_max_y());

        this->below = below;
    }
    void set_left(std::shared_ptr<const T_grid_rect> left)
    {
        assert(this->get_min_y() == above->get_min_y());
        assert(this->get_max_y() == above->get_max_y());
        assert(this->get_min_x() == above->get_max_x());

        this->left = left;
    }
    void set_right(std::shared_ptr<const T_grid_rect> right)
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
