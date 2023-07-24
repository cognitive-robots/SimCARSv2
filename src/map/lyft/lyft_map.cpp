
#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>

#include <lz4_stream.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <filesystem>
#include <fstream>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

LyftMap::LyftMap() : map_grid_dict(geometry::Vec(0, 0), 100) {}

LyftMap::~LyftMap()
{
    clear();
}

ILane const* LyftMap::get_lane(uint64_t id) const
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

structures::IArray<ILane const*>* LyftMap::get_lanes(structures::IArray<uint64_t> const *ids) const
{
    structures::IArray<ILane const*> *lane_array =
            new structures::stl::STLStackArray<ILane const*>(ids->count());

    for (size_t i = 0; i < ids->count(); ++i)
    {
        (*lane_array)[i] = get_lane((*ids)[i]);
    }

    return lane_array;
}

structures::IArray<ILane const*>* LyftMap::get_encapsulating_lanes(geometry::Vec point) const
{
    if (map_grid_dict.contains(point))
    {
        MapGridRect *map_grid_rect = map_grid_dict[point];
        if (map_grid_rect != nullptr)
        {
            return map_grid_rect->get_encapsulating_lanes(point);
        }
    }

    return new structures::stl::STLStackArray<ILane const*>;
}

structures::IArray<ILane const*>* LyftMap::get_lanes_in_range(geometry::Vec point,
                                                              FP_DATA_TYPE distance) const
{
    structures::IArray<MapGridRect*> *map_grid_rects =
            map_grid_dict.chebyshev_grid_rects_in_range(point, distance);

    structures::stl::STLSet<ILane const*> lanes;

    for (size_t i = 0; i < map_grid_rects->count(); ++i)
    {
        lanes.union_with((*map_grid_rects)[i]->get_lanes());
    }

    structures::IStackArray<ILane const*> *lane_array =
            new structures::stl::STLStackArray<ILane const*>(lanes.count());

    lanes.get_array(lane_array);

    delete map_grid_rects;

    return lane_array;
}

void LyftMap::save(std::string const &output_file_path_str) const
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

void LyftMap::clear()
{
    structures::IArray<LyftLane*> const *lane_array =
            id_to_lane_dict.get_values();
    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        delete (*lane_array)[i];
    }
}

void LyftMap::load(std::string const &input_file_path_str)
{
    std::filesystem::path input_file_path(input_file_path_str);

    if (!std::filesystem::is_regular_file(input_file_path))
    {
        throw std::invalid_argument("Input file path '" + input_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::ifstream input_filestream(input_file_path, std::ios_base::binary);

    lz4_stream::istream input_lz4_stream(input_filestream);
    rapidjson::BasicIStreamWrapper input_lz4_json_stream(input_lz4_stream);

    rapidjson::Document json_document;
    json_document.ParseStream(input_lz4_json_stream);

    clear();

    rapidjson::Value::Object lane_id_to_lane_data = json_document["id_lane_dict"].GetObject();

    for (rapidjson::Value::ConstMemberIterator lane_id_to_lane_itr =
         lane_id_to_lane_data.MemberBegin();
         lane_id_to_lane_itr != lane_id_to_lane_data.MemberEnd(); ++lane_id_to_lane_itr)
    {
        size_t i, j;

        std::string const &raw_lane_id = lane_id_to_lane_itr->name.GetString();
        uint64_t lane_id = 0;
        for (i = 0; i < 4 && i < raw_lane_id.length(); ++i)
        {
            lane_id |= (raw_lane_id[i] << (8 * i));
        }

        rapidjson::Value::ConstObject json_lane_data = lane_id_to_lane_itr->value.GetObject();

        std::string const left_adjacent_raw_lane_id(
                    json_lane_data["adjacent_left_id"].GetString(),
                json_lane_data["adjacent_left_id"].GetStringLength());
        uint64_t left_adjacent_lane_id = 0;
        for (i = 0; i < 4 && i < left_adjacent_raw_lane_id.length(); ++i)
        {
            left_adjacent_lane_id |= (left_adjacent_raw_lane_id[i] << (8 * i));
        }

        std::string const right_adjacent_raw_lane_id(
                    json_lane_data["adjacent_right_id"].GetString(),
                json_lane_data["adjacent_right_id"].GetStringLength());
        uint64_t right_adjacent_lane_id = 0;
        for (i = 0; i < 4 && i < right_adjacent_raw_lane_id.length(); ++i)
        {
            right_adjacent_lane_id |= (right_adjacent_raw_lane_id[i] << (8 * i));
        }

        rapidjson::Value::ConstArray const &fore_lane_data =
                json_lane_data["ahead_ids"].GetArray();
        size_t const fore_lane_count = fore_lane_data.Capacity();
        structures::IStackArray<uint64_t>* fore_lane_ids =
                new structures::stl::STLStackArray<uint64_t>(fore_lane_count);
        for (i = 0; i < fore_lane_count; ++i)
        {
            std::string const fore_lane_raw_lane_id(fore_lane_data[i].GetString(),
                                                    fore_lane_data[i].GetStringLength());
            uint64_t fore_lane_id = 0;
            for (j = 0; j < 4 && j < fore_lane_raw_lane_id.length(); ++j)
            {
                fore_lane_id |= (fore_lane_raw_lane_id[j] << (8 * j));
            }
            (*fore_lane_ids)[i] = fore_lane_id;
        }

        rapidjson::Value::ConstArray const &aft_lane_data =
                json_lane_data["behind_ids"].GetArray();
        size_t const aft_lane_count = aft_lane_data.Capacity();
        structures::IStackArray<uint64_t>* aft_lane_ids =
                new structures::stl::STLStackArray<uint64_t>(aft_lane_count);
        for (i = 0; i < aft_lane_count; ++i)
        {
            std::string const aft_lane_raw_lane_id(aft_lane_data[i].GetString(),
                                                   aft_lane_data[i].GetStringLength());
            uint64_t aft_lane_id = 0;
            for (j = 0; j < 4 && j < aft_lane_raw_lane_id.length(); ++j)
            {
                aft_lane_id |= (aft_lane_raw_lane_id[j] << (8 * j));
            }
            (*aft_lane_ids)[i] = aft_lane_id;
        }

        LyftLane *lane = new LyftLane(lane_id, left_adjacent_lane_id, right_adjacent_lane_id,
                                      fore_lane_ids, aft_lane_ids, this, json_lane_data);
        id_to_lane_dict.update(lane_id, lane);

        geometry::Rect const &lane_bounding_box = lane->get_bounding_box();

        map_grid_dict.chebyshev_proliferate(
            lane_bounding_box.get_origin(),
            lane_bounding_box.get_half_width(),
            lane_bounding_box.get_half_height());

        structures::IArray<MapGridRect*> *map_grid_rects =
                map_grid_dict.chebyshev_grid_rects_in_range(
                    lane_bounding_box.get_origin(),
                    lane_bounding_box.get_half_width(),
                    lane_bounding_box.get_half_height());

        for (size_t i = 0; i < map_grid_rects->count(); ++i)
        {
            (*map_grid_rects)[i]->insert_lane(lane);
        }

        delete map_grid_rects;
    }
}

}
}
}
}
