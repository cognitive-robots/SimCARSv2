#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/grid_rect.hpp>

#include <cmath>

#define GOLDEN_RATIO_MAGIC_NUM 0x9e3779b9

namespace ori
{
namespace simcars
{
namespace geometry
{

class VecHasher
{
    std::hash<FP_DATA_TYPE> hasher;

public:
    std::size_t operator()(Vec const &key) const
    {
        size_t key_hash = hasher(key.x());
        key_hash ^= hasher(key.y()) + GOLDEN_RATIO_MAGIC_NUM + (key_hash << 6) + (key_hash >> 2);
        return key_hash;
    }
};

template <typename V_grid_rect>
class GridDictionary : public virtual structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>
{
    Vec origin;
    FP_DATA_TYPE spacing;

    Vec round(Vec const &point) const
    {
        Vec rounded_point = point;
        rounded_point = (rounded_point - origin) / spacing;
        rounded_point = rounded_point.array().round().matrix();
        rounded_point = (rounded_point * spacing) + origin;
        return rounded_point;
    }

public:
    GridDictionary(Vec origin, FP_DATA_TYPE spacing, size_t bin_count = 10000)
        : structures::stl::STLDictionary<Vec, *V_grid_rect, VecHasher>(bin_count), origin(origin), spacing(spacing) {}
    GridDictionary(GridDictionary const &grid_dictionary)
        : structures::stl::STLDictionary<Vec, *V_grid_rect, VecHasher>(grid_dictionary),
          origin(grid_dictionary.origin), spacing(grid_dictionary.spacing) {}
    ~GridDictionary() override
    {
        static_assert(std::is_base_of<GridRect<V_grid_rect>, V_grid_rect>::value, "V_grid_rect is not derived from GridRect");
    }

    bool contains(Vec const &key) const override
    {
        return structures::stl::STLDictionary<Vec, *V_grid_rect, VecHasher>::contains(round(key));
    }

    V_grid_rect* const& operator [](Vec const &key) const override
    {
        return structures::stl::STLDictionary<Vec, *V_grid_rect, VecHasher>::operator [](round(key));
    }

    structures::IArray<geometry::Vec>* chebyshev_grid_points_in_range(Vec const &key, FP_DATA_TYPE distance) const
    {
        return chebyshev_grid_points_in_range(key, distance, distance);
    }
    structures::IArray<geometry::Vec>* chebyshev_grid_points_in_range(Vec const &key, FP_DATA_TYPE x_distance, FP_DATA_TYPE y_distance) const
    {
        Vec rounded_point = round(key);
        Vec rounded_top_right_point = round(key + Vec(x_distance, y_distance));
        Vec rounded_bottom_left_point = round(key - Vec(x_distance, y_distance));
        int i, j;
        int i_min = (int)((rounded_bottom_left_point.x() - rounded_point.x()) / spacing);
        int i_max = (int)((rounded_top_right_point.x() - rounded_point.x()) / spacing);
        int j_min = (int)((rounded_bottom_left_point.y() - rounded_point.y()) / spacing);
        int j_max = (int)((rounded_top_right_point.y() - rounded_point.y()) / spacing);

        structures::IArray<geometry::Vec> *grid_points = new structures::stl::STLStackArray<geometry::Vec>((1 + i_max - i_min) * (1 + j_max - j_min));

        for (i = i_min; i <= i_max; ++i)
        {
            Vec i_adjustment(i * spacing, 0.0f);
            for (j = j_min; j <= j_max; ++j)
            {
                Vec j_adjustment(0.0f, j * spacing);
                Vec current_point = rounded_point + i_adjustment + j_adjustment;
                (*grid_points)[(j - j_min) + (i - i_min) * (1 + j_max - j_min)] = current_point;
            }
        }

        return grid_points;
    }
    structures::IArray<V_grid_rect*>* chebyshev_grid_rects_in_range(Vec const &key, FP_DATA_TYPE distance) const
    {
        return chebyshev_grid_rects_in_range(key, distance, distance);
    }
    structures::IArray<V_grid_rect*>* chebyshev_grid_rects_in_range(Vec const &key, FP_DATA_TYPE x_distance, FP_DATA_TYPE y_distance) const
    {
        structures::IArray<geometry::Vec> *grid_points = chebyshev_grid_points_in_range(key, x_distance, y_distance);
        structures::stl::STLStackArray<V_grid_rect*> *grid_rects = new structures::stl::STLStackArray<V_grid_rect*>();
        size_t i;
        for (i = 0; i < grid_points->count(); ++i)
        {
            if (structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>::contains((*grid_points)[i]))
            {
                grid_rects->push_back(
                            structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>::operator [](
                                (*grid_points)[i]));
            }
        }

        return grid_rects;
    }

    void update(Vec const &key, V_grid_rect* const &val) override
    {
        structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>::update(round(key), val);
    }
    void erase(Vec const &key) override
    {
        structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>::erase(round(key));
    }

    void chebyshev_proliferate(Vec const &key, FP_DATA_TYPE distance)
    {
        chebyshev_proliferate(key, distance, distance);
    }
    void chebyshev_proliferate(Vec const &key, FP_DATA_TYPE x_distance, FP_DATA_TYPE y_distance)
    {
        structures::IArray<geometry::Vec> *grid_points = chebyshev_grid_points_in_range(key, x_distance, y_distance);
        size_t i;
        for (i = 0; i < grid_points->count(); ++i)
        {
            if (!structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>::contains((*grid_points)[i]))
            {
                structures::stl::STLDictionary<Vec, V_grid_rect*, VecHasher>::update(
                            (*grid_points)[i],
                            new V_grid_rect((*grid_points)[i], spacing));
            }
        }
    }
};

}
}
}
