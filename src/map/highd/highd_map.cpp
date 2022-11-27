
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/living_lane_stack_array.hpp>
#include <ori/simcars/map/living_traffic_light_stack_array.hpp>
#include <ori/simcars/map/highd/highd_lane.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>

#include <rapidcsv.h>

namespace ori
{
namespace simcars
{
namespace map
{
namespace highd
{

void HighDMap::save_virt(std::ofstream &output_filestream) const
{
    throw utils::NotImplementedException();
}

void HighDMap::load_virt(std::ifstream &input_filestream)
{
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

    id_to_lane_dict = new structures::stl::STLDictionary<uint8_t, HighDLane*>;

    stray_ghosts = new structures::stl::STLSet<IMapObject<uint8_t> const*>;

    for (size_t i = 0; i < upper_lane_markings.count() - 1; ++i)
    {
        uint8_t const lane_id = i + 1;

        uint8_t left_adjacent_lane_id;
        if (lane_id != upper_lane_markings.count() - 2)
        {
            left_adjacent_lane_id = lane_id + 1;
        }
        else
        {
            left_adjacent_lane_id = 0;
        }

        uint8_t right_adjacent_lane_id;
        if (lane_id != 1)
        {
            right_adjacent_lane_id = lane_id - 1;
        }
        else
        {
            right_adjacent_lane_id = 0;
        }

        id_to_lane_dict->update(lane_id,
                                new HighDLane(lane_id, this, upper_lane_markings[i], upper_lane_markings[i + 1],
                                              true, left_adjacent_lane_id, right_adjacent_lane_id));
    }

    for (size_t i = 0; i < lower_lane_markings.count() - 1; ++i)
    {
        uint8_t const lane_id = i + upper_lane_markings.count();

        uint8_t left_adjacent_lane_id;
        if (lane_id != 1)
        {
            left_adjacent_lane_id = lane_id - 1;
        }
        else
        {
            left_adjacent_lane_id = 0;
        }

        uint8_t right_adjacent_lane_id;
        if (lane_id != upper_lane_markings.count() - 2)
        {
            right_adjacent_lane_id = lane_id + 1;
        }
        else
        {
            right_adjacent_lane_id = 0;
        }

        id_to_lane_dict->update(lane_id,
                                new HighDLane(lane_id, this, lower_lane_markings[i], lower_lane_markings[i + 1],
                                              false, left_adjacent_lane_id, right_adjacent_lane_id));
    }
}

HighDMap::~HighDMap()
{
    size_t i;

    structures::IArray<HighDLane*> const *lane_array =
            id_to_lane_dict->get_values();
    for (i = 0; i < lane_array->count(); ++i)
    {
        delete (*lane_array)[i];
    }
    delete id_to_lane_dict;

    structures::IArray<IMapObject<uint8_t> const*> const *ghost_array =
            stray_ghosts->get_array();
    for (i = 0; i < ghost_array->count(); ++i)
    {
        delete (*ghost_array)[i];
    }
    delete stray_ghosts;
}

ILane<uint8_t> const* HighDMap::get_lane(uint8_t id) const
{
    try
    {
        return (*id_to_lane_dict)[id];
    }
    catch (std::out_of_range const&)
    {
        throw IMap::ObjectNotFound();
    }
}

ILaneArray<uint8_t> const* HighDMap::get_encapsulating_lanes(geometry::Vec point) const
{
    map::LivingLaneStackArray<uint8_t> *encapsulating_lanes = new map::LivingLaneStackArray<uint8_t>;

    structures::IArray<HighDLane*> const *lane_array =
            id_to_lane_dict->get_values();
    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        if ((*lane_array)[i]->check_encapsulation(point))
        {
            encapsulating_lanes->push_back((*lane_array)[i]);
        }
    }

    return encapsulating_lanes;
}

ILaneArray<uint8_t> const* HighDMap::get_lanes(structures::IArray<uint8_t> const *ids) const
{
    const size_t lane_count = ids->count();
    ILaneArray<uint8_t> *lanes =
            new LivingLaneStackArray<uint8_t>(lane_count);

    for (size_t i = 0; i < lane_count; ++i)
    {
        (*lanes)[i] = get_lane((*ids)[i]);
    }

    return lanes;
}

ILaneArray<uint8_t> const* HighDMap::get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    // WARNING: This doesn't actually calculate which lanes are in range, it just returns them all,
    // mainly because the primary use of this method is for rendering, and nearly all the High-D scenes
    // are comprised of ~6 lanes which are always in view simultaneously
    structures::IArray<HighDLane*> const *lane_array =
            id_to_lane_dict->get_values();
    ILaneArray<uint8_t> *lanes =
            new LivingLaneStackArray<uint8_t>(id_to_lane_dict->count());
    cast_array<HighDLane*, ILane<uint8_t> const*>(*lane_array, *lanes);
    return lanes;
}

ITrafficLight<uint8_t> const* HighDMap::get_traffic_light(uint8_t id) const
{
    throw IMap::ObjectNotFound();
}

ITrafficLightArray<uint8_t> const* HighDMap::get_traffic_lights(structures::IArray<uint8_t> const *ids) const
{
    size_t const traffic_light_count = ids->count();
    ITrafficLightArray<uint8_t> *traffic_lights =
            new LivingTrafficLightStackArray<uint8_t>(traffic_light_count);

    for (size_t i = 0; i < traffic_light_count; ++i)
    {
        (*traffic_lights)[i] = get_traffic_light((*ids)[i]);
    }

    return traffic_lights;
}

ITrafficLightArray<uint8_t> const* HighDMap::get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    return new LivingTrafficLightStackArray<uint8_t>;
}


void HighDMap::register_stray_ghost(IMapObject<uint8_t> const *ghost) const
{
    stray_ghosts->insert(ghost);
}

void HighDMap::unregister_stray_ghost(IMapObject<uint8_t> const *ghost) const
{
    stray_ghosts->erase(ghost);
}

HighDMap* HighDMap::shallow_copy() const
{
    HighDMap *map_copy = new HighDMap;

    map_copy->id_to_lane_dict =
            new structures::stl::STLDictionary<uint8_t, HighDLane*>(
                this->id_to_lane_dict);

    map_copy->stray_ghosts = new structures::stl::STLSet<IMapObject<uint8_t> const*>(this->stray_ghosts);

    return map_copy;
}

}
}
}
}
