

#include <ori/simcars/map/highd/highd_map.hpp>

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
namespace highd
{

HighDMap::~HighDMap()
{
    clear();
}

ILane const* HighDMap::get_lane(uint64_t id) const
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

structures::IArray<ILane const*>* HighDMap::get_lanes(structures::IArray<uint64_t> const *ids) const
{
    structures::IArray<ILane const*> *lane_array =
            new structures::stl::STLStackArray<ILane const*>(ids->count());

    for (size_t i = 0; i < ids->count(); ++i)
    {
        (*lane_array)[i] = get_lane((*ids)[i]);
    }

    return lane_array;
}

structures::IArray<ILane const*>* HighDMap::get_encapsulating_lanes(geometry::Vec point) const
{
    structures::IStackArray<ILane const*> *encapsulating_lanes =
            new structures::stl::STLStackArray<ILane const*>;

    structures::IArray<HighDLane*> const *lane_array =
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

structures::IArray<ILane const*>* HighDMap::get_lanes_in_range(geometry::Vec point,
                                                               FP_DATA_TYPE distance) const
{
    // WARNING: This doesn't actually calculate which lanes are in range, it just returns them all,
    // mainly because the primary use of this method is for rendering, and nearly all the
    // High-D scenes are comprised of ~6 lanes which are always in view simultaneously
    structures::IArray<HighDLane*> const *original_lane_array = id_to_lane_dict.get_values();
    structures::IArray<ILane const*> *new_lane_array =
            new structures::stl::STLStackArray<ILane const*>(id_to_lane_dict.count());
    cast_array<HighDLane*, ILane const*>(*original_lane_array, *new_lane_array);
    return new_lane_array;
}

void HighDMap::save(std::string const &output_file_path_str) const
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

void HighDMap::clear()
{
    structures::IArray<HighDLane*> const *lane_array =
            id_to_lane_dict.get_values();
    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        delete (*lane_array)[i];
    }
}

void HighDMap::load(std::string const &input_file_path_str)
{
    std::filesystem::path input_file_path(input_file_path_str);

    if (!std::filesystem::is_regular_file(input_file_path))
    {
        throw std::invalid_argument("Input file path '" + input_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::ifstream input_filestream(input_file_path, std::ios_base::binary);

    rapidcsv::Document csv_document(input_filestream);

    std::string upper_lane_markings_str = csv_document.GetCell<std::string>("upperLaneMarkings", 0);
    std::stringstream upper_lane_markings_str_stream(upper_lane_markings_str);
    structures::stl::STLStackArray<FP_DATA_TYPE> upper_lane_markings;
    std::string upper_lane_marking_str;
    while (getline(upper_lane_markings_str_stream, upper_lane_marking_str, ';'))
    {
        upper_lane_markings.push_back(std::stod(upper_lane_marking_str));
    }

    std::string lower_lane_markings_str = csv_document.GetCell<std::string>("lowerLaneMarkings", 0);
    std::stringstream lower_lane_markings_str_stream(lower_lane_markings_str);
    structures::stl::STLStackArray<FP_DATA_TYPE> lower_lane_markings;
    std::string lower_lane_marking_str;
    while (getline(lower_lane_markings_str_stream, lower_lane_marking_str, ';'))
    {
        lower_lane_markings.push_back(std::stod(lower_lane_marking_str));
    }

    clear();

    for (size_t i = 0; i < upper_lane_markings.count() - 1; ++i)
    {
        uint64_t const lane_id = i + 1;

        uint64_t left_adjacent_lane_id;
        if (i != upper_lane_markings.count() - 2)
        {
            left_adjacent_lane_id = lane_id + 1;
        }
        else
        {
            left_adjacent_lane_id = 0;
        }

        uint64_t right_adjacent_lane_id;
        if (i != 0)
        {
            right_adjacent_lane_id = lane_id - 1;
        }
        else
        {
            right_adjacent_lane_id = 0;
        }

        id_to_lane_dict.update(lane_id,
                                new HighDLane(lane_id, left_adjacent_lane_id,
                                              right_adjacent_lane_id, this, upper_lane_markings[i],
                                              upper_lane_markings[i + 1], true));
    }

    for (size_t i = 0; i < lower_lane_markings.count() - 1; ++i)
    {
        uint64_t const lane_id = i + upper_lane_markings.count();

        uint64_t left_adjacent_lane_id;
        if (i != 0)
        {
            left_adjacent_lane_id = lane_id - 1;
        }
        else
        {
            left_adjacent_lane_id = 0;
        }

        uint64_t right_adjacent_lane_id;
        if (i != lower_lane_markings.count() - 2)
        {
            right_adjacent_lane_id = lane_id + 1;
        }
        else
        {
            right_adjacent_lane_id = 0;
        }

        id_to_lane_dict.update(lane_id,
                                new HighDLane(lane_id, left_adjacent_lane_id,
                                              right_adjacent_lane_id, this, lower_lane_markings[i],
                                              lower_lane_markings[i + 1], false));
    }
}

}
}
}
}
