
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/living_lane_stack_array.hpp>
#include <ori/simcars/map/living_traffic_light_stack_array.hpp>
#include <ori/simcars/map/plg/plg_lane.hpp>
#include <ori/simcars/map/plg/plg_map.hpp>

#include <rapidcsv.h>

namespace ori
{
namespace simcars
{
namespace map
{
namespace plg
{

void PLGMap::save_virt(std::ofstream &output_filestream) const
{
    throw utils::NotImplementedException();
}

void PLGMap::load_virt(std::ifstream &input_filestream)
{
    rapidcsv::Document csv_document(input_filestream, rapidcsv::LabelParams(-1, -1),
                                    rapidcsv::SeparatorParams(' '));

    structures::stl::STLDictionary<uint8_t, uint32_t> id_to_vertex_count_dict;

    std::vector<FP_DATA_TYPE> x_values = csv_document.GetColumn<FP_DATA_TYPE>(0);
    std::vector<FP_DATA_TYPE> y_values = csv_document.GetColumn<FP_DATA_TYPE>(1);
    std::vector<FP_DATA_TYPE> lane_ids = csv_document.GetColumn<FP_DATA_TYPE>(2);

    uint8_t lane_id;
    size_t i;
    for (i = 0; i < lane_ids.size(); ++i)
    {
        lane_id = uint8_t(lane_ids[i]);

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

    structures::stl::STLDictionary<uint8_t, geometry::Vecs*> id_to_vertices_dict;

    for (i = 0; i < lane_ids.size(); ++i)
    {
        lane_id = uint8_t(lane_ids[i]);

        uint32_t const vertices_remaining = id_to_vertex_count_dict[lane_id];

        if (!id_to_vertices_dict.contains(lane_id))
        {
            id_to_vertices_dict.update(
                        lane_id,
                        new geometry::Vecs(2, vertices_remaining));
        }

        geometry::Vecs* const vertices = id_to_vertices_dict[lane_id];

        uint32_t const current_vertex = vertices->cols() - vertices_remaining;

        (*vertices)(0, current_vertex) = x_values[i];
        (*vertices)(1, current_vertex) = y_values[i];

        id_to_vertex_count_dict.update(lane_id, vertices_remaining - 1);
    }

    id_to_lane_dict = new structures::stl::STLDictionary<uint8_t, PLGLane*>;

    stray_ghosts = new structures::stl::STLSet<IMapObject<uint8_t> const*>;

    structures::IArray<uint8_t> const *unique_lane_ids = id_to_vertices_dict.get_keys();

    for (i = 0; i < unique_lane_ids->count(); ++i)
    {
        lane_id = (*unique_lane_ids)[i];
        geometry::Vecs* const vertices = id_to_vertices_dict[lane_id];

        PLGLane *lane = new PLGLane(lane_id, this, vertices);

        id_to_lane_dict->update(lane_id, lane);

        delete vertices;
    }
}

PLGMap::~PLGMap()
{
    size_t i;

    structures::IArray<PLGLane*> const *lane_array =
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

ILane<uint8_t> const* PLGMap::get_lane(uint8_t id) const
{
    if (id_to_lane_dict->contains(id))
    {
        return (*id_to_lane_dict)[id];
    }
    else
    {
        return nullptr;
    }
}

ILaneArray<uint8_t> const* PLGMap::get_encapsulating_lanes(geometry::Vec point) const
{
    map::LivingLaneStackArray<uint8_t> *encapsulating_lanes = new map::LivingLaneStackArray<uint8_t>;

    structures::IArray<PLGLane*> const *lane_array =
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

ILaneArray<uint8_t> const* PLGMap::get_lanes(structures::IArray<uint8_t> const *ids) const
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

ILaneArray<uint8_t> const* PLGMap::get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    // WARNING: This doesn't actually calculate which lanes are in range, it just returns them all
    structures::IArray<PLGLane*> const *lane_array =
            id_to_lane_dict->get_values();
    ILaneArray<uint8_t> *lanes =
            new LivingLaneStackArray<uint8_t>(id_to_lane_dict->count());
    cast_array<PLGLane*, ILane<uint8_t> const*>(*lane_array, *lanes);
    return lanes;
}

ITrafficLight<uint8_t> const* PLGMap::get_traffic_light(uint8_t id) const
{
    return nullptr;
}

ITrafficLightArray<uint8_t> const* PLGMap::get_traffic_lights(structures::IArray<uint8_t> const *ids) const
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

ITrafficLightArray<uint8_t> const* PLGMap::get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    return new LivingTrafficLightStackArray<uint8_t>;
}


void PLGMap::register_stray_ghost(IMapObject<uint8_t> const *ghost) const
{
    stray_ghosts->insert(ghost);
}

void PLGMap::unregister_stray_ghost(IMapObject<uint8_t> const *ghost) const
{
    stray_ghosts->erase(ghost);
}

PLGMap* PLGMap::shallow_copy() const
{
    PLGMap *map_copy = new PLGMap;

    map_copy->id_to_lane_dict =
            new structures::stl::STLDictionary<uint8_t, PLGLane*>(
                this->id_to_lane_dict);

    map_copy->stray_ghosts = new structures::stl::STLSet<IMapObject<uint8_t> const*>(this->stray_ghosts);

    return map_copy;
}

}
}
}
}
