

#include <ori/simcars/map/plg/plg_map.hpp>

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>

#include <rapidcsv.h>

#include <filesystem>
#include <fstream>

namespace ori
{
namespace simcars
{
namespace map
{
namespace plg
{

PLGMap::~PLGMap()
{
    clear();
}

geometry::Vec PLGMap::get_map_centre() const
{
    structures::IArray<PLGLane*> const *lane_array = id_to_lane_dict.get_values();

    FP_DATA_TYPE min_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_y = std::numeric_limits<FP_DATA_TYPE>::min();

    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        geometry::Rect const &rect = (*lane_array)[i]->get_bounding_box();
        min_x = std::min(rect.get_min_x(), min_x);
        max_x = std::max(rect.get_max_x(), max_x);
        min_y = std::min(rect.get_min_y(), min_y);
        max_y = std::max(rect.get_max_y(), max_y);
    }

    return geometry::Vec((min_x + max_x) / 2, (min_y + max_y) / 2);
}

geometry::Rect PLGMap::get_bounding_box() const
{
    structures::IArray<PLGLane*> const *lane_array = id_to_lane_dict.get_values();

    FP_DATA_TYPE min_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_y = std::numeric_limits<FP_DATA_TYPE>::min();

    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        geometry::Rect const &rect = (*lane_array)[i]->get_bounding_box();
        min_x = std::min(rect.get_min_x(), min_x);
        max_x = std::max(rect.get_max_x(), max_x);
        min_y = std::min(rect.get_min_y(), min_y);
        max_y = std::max(rect.get_max_y(), max_y);
    }

    return geometry::Rect(min_x, min_y, max_x, max_y);
}

FP_DATA_TYPE PLGMap::get_max_dim_size() const
{
    structures::IArray<PLGLane*> const *lane_array = id_to_lane_dict.get_values();

    FP_DATA_TYPE min_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_y = std::numeric_limits<FP_DATA_TYPE>::min();

    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        geometry::Rect const &rect = (*lane_array)[i]->get_bounding_box();
        min_x = std::min(rect.get_min_x(), min_x);
        max_x = std::max(rect.get_max_x(), max_x);
        min_y = std::min(rect.get_min_y(), min_y);
        max_y = std::max(rect.get_max_y(), max_y);
    }

    return std::max(max_x - min_x, max_y - min_y);
}

ILane const* PLGMap::get_lane(uint64_t id) const
{
    if (id_to_lane_dict.contains(id))
    {
        return id_to_lane_dict[id];
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<ILane const*>* PLGMap::get_lanes(structures::IArray<uint64_t> const *ids) const
{
    structures::IArray<ILane const*> *lane_array =
            new structures::stl::STLStackArray<ILane const*>(ids->count());

    for (size_t i = 0; i < ids->count(); ++i)
    {
        (*lane_array)[i] = get_lane((*ids)[i]);
    }

    return lane_array;
}

structures::IArray<ILane const*>* PLGMap::get_encapsulating_lanes(geometry::Vec point) const
{
    structures::IStackArray<ILane const*> *encapsulating_lanes =
            new structures::stl::STLStackArray<ILane const*>;

    structures::IArray<PLGLane*> const *lane_array =
            id_to_lane_dict.get_values();
    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        if ((*lane_array)[i]->check_encapsulation(point))
        {
            encapsulating_lanes->push_back((*lane_array)[i]);
        }
    }

    return encapsulating_lanes;
}

structures::IArray<ILane const*>* PLGMap::get_lanes_in_range(geometry::Vec point,
                                                             FP_DATA_TYPE distance) const
{
    // WARNING: This doesn't actually calculate which lanes are in range, it just returns them all,
    // mainly because the primary use of this method is for rendering, and nearly all the
    // High-D scenes are comprised of ~6 lanes which are always in view simultaneously
    structures::IArray<PLGLane*> const *original_lane_array = id_to_lane_dict.get_values();
    structures::IArray<ILane const*> *new_lane_array =
            new structures::stl::STLStackArray<ILane const*>(id_to_lane_dict.count());
    cast_array<PLGLane*, ILane const*>(*original_lane_array, *new_lane_array);
    return new_lane_array;
}

void PLGMap::save(std::string const &output_file_path_str) const
{
    throw utils::NotImplementedException();

    /*
    std::filesystem::path output_file_path(output_file_path_str);

    if (!std::filesystem::is_directory(output_file_path.parent_path()))
    {
        throw std::invalid_argument("Output file path directory '" +
                                    output_file_path.parent_path().string() +
                                    "' does not indicate a valid directory");
    }

    std::ofstream output_filestream(output_file_path, std::ios_base::binary);
    */
}

void PLGMap::clear()
{
    structures::IArray<PLGLane*> const *lane_array =
            id_to_lane_dict.get_values();
    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        delete (*lane_array)[i];
    }
}

void PLGMap::load(std::string const &input_file_path_str)
{
    std::filesystem::path input_file_path(input_file_path_str);

    if (!std::filesystem::is_regular_file(input_file_path))
    {
        throw std::invalid_argument("Input file path '" + input_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::ifstream input_filestream(input_file_path, std::ios_base::binary);

    rapidcsv::Document csv_document(input_filestream, rapidcsv::LabelParams(-1, -1),
                                    rapidcsv::SeparatorParams(' '));

    structures::stl::STLDictionary<uint64_t, uint32_t> id_to_vertex_count_dict;

    std::vector<FP_DATA_TYPE> x_values = csv_document.GetColumn<FP_DATA_TYPE>(0);
    std::vector<FP_DATA_TYPE> y_values = csv_document.GetColumn<FP_DATA_TYPE>(1);
    std::vector<FP_DATA_TYPE> lane_ids = csv_document.GetColumn<FP_DATA_TYPE>(2);

    uint64_t lane_id = 0;
    size_t i;
    for (i = 0; i < lane_ids.size(); ++i)
    {
        if (i > 0 && lane_ids[i] != lane_ids[i - 1])
        {
            lane_id++;
        }

        if (id_to_vertex_count_dict.contains(lane_id))
        {
            size_t current_vertex_count = id_to_vertex_count_dict[lane_id];
            id_to_vertex_count_dict.update(lane_id, current_vertex_count + 1);
        }
        else
        {
            id_to_vertex_count_dict.update(lane_id, 1);
        }
    }

    structures::stl::STLDictionary<uint64_t, geometry::Vecs*> id_to_vertices_dict;

    lane_id = 0;
    for (i = 0; i < lane_ids.size(); ++i)
    {
        if (i > 0 && lane_ids[i] != lane_ids[i - 1])
        {
            lane_id++;
        }

        uint32_t const vertices_remaining = id_to_vertex_count_dict[lane_id];

        if (!id_to_vertices_dict.contains(lane_id))
        {
            if (vertices_remaining < 2)
            {
                continue;
            }
            else
            {
                id_to_vertices_dict.update(
                            lane_id,
                            new geometry::Vecs(2, vertices_remaining));
            }
        }

        geometry::Vecs* const vertices = id_to_vertices_dict[lane_id];

        uint32_t const current_vertex = vertices->cols() - vertices_remaining;

        (*vertices)(0, current_vertex) = x_values[i];
        (*vertices)(1, current_vertex) = y_values[i];

        id_to_vertex_count_dict.update(lane_id, vertices_remaining - 1);
    }

    clear();

    structures::IArray<uint64_t> const *unique_lane_ids = id_to_vertices_dict.get_keys();

    for (i = 0; i < unique_lane_ids->count(); ++i)
    {
        lane_id = (*unique_lane_ids)[i];
        geometry::Vecs* const vertices = id_to_vertices_dict[lane_id];

        PLGLane *lane = new PLGLane(lane_id, this, vertices);

        id_to_lane_dict.update(lane_id, lane);

        delete vertices;
    }
}

}
}
}
}
