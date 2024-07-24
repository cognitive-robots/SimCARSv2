

#include <ori/simcars/map/laneletd/laneletd_map.hpp>

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <rapidcsv.h>

#include <filesystem>
#include <fstream>

namespace ori
{
namespace simcars
{
namespace map
{
namespace laneletd
{

LaneletDMap::~LaneletDMap()
{
    clear();
}

geometry::Vec LaneletDMap::get_map_centre() const
{
    structures::IArray<LaneletDLane*> const *lane_array = id_to_lane_dict.get_values();

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

geometry::Rect LaneletDMap::get_bounding_box() const
{
    structures::IArray<LaneletDLane*> const *lane_array = id_to_lane_dict.get_values();

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

FP_DATA_TYPE LaneletDMap::get_max_dim_size() const
{
    structures::IArray<LaneletDLane*> const *lane_array = id_to_lane_dict.get_values();

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

ILane const* LaneletDMap::get_lane(uint64_t id) const
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

structures::IArray<ILane const*>* LaneletDMap::get_lanes(structures::IArray<uint64_t> const *ids) const
{
    structures::IArray<ILane const*> *lane_array =
            new structures::stl::STLStackArray<ILane const*>(ids->count());

    for (size_t i = 0; i < ids->count(); ++i)
    {
        (*lane_array)[i] = get_lane((*ids)[i]);
    }

    return lane_array;
}

structures::IArray<ILane const*>* LaneletDMap::get_encapsulating_lanes(geometry::Vec point) const
{
    structures::IStackArray<ILane const*> *encapsulating_lanes =
            new structures::stl::STLStackArray<ILane const*>;

    structures::IArray<LaneletDLane*> const *lane_array =
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

structures::IArray<ILane const*>* LaneletDMap::get_lanes_in_range(geometry::Vec point,
                                                                 FP_DATA_TYPE distance) const
{
    // WARNING: This doesn't actually calculate which lanes are in range, it just returns them all,
    // mainly because the primary use of this method is for rendering, and nearly all the
    // High-D scenes are comprised of ~6 lanes which are always in view simultaneously
    structures::IArray<LaneletDLane*> const *original_lane_array = id_to_lane_dict.get_values();
    structures::IArray<ILane const*> *new_lane_array =
            new structures::stl::STLStackArray<ILane const*>(id_to_lane_dict.count());
    cast_array<LaneletDLane*, ILane const*>(*original_lane_array, *new_lane_array);
    return new_lane_array;
}

void LaneletDMap::save(std::string const &output_file_path_str,
                       std::string const &recording_meta_file_path_str) const
{
    throw utils::NotImplementedException();
}

void LaneletDMap::clear()
{
    structures::IArray<LaneletDLane*> const *lane_array =
            id_to_lane_dict.get_values();
    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        delete (*lane_array)[i];
    }
}

void LaneletDMap::load(std::string const &input_file_path_str,
                       std::string const &recording_meta_file_path_str)
{
    std::filesystem::path input_file_path(input_file_path_str);
    if (!std::filesystem::is_regular_file(input_file_path))
    {
        throw std::invalid_argument("Input file path '" + input_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path recording_meta_file_path(recording_meta_file_path_str);
    if (!std::filesystem::is_regular_file(recording_meta_file_path))
    {
        throw std::invalid_argument("Recording meta file path '" + recording_meta_file_path_str +
                                    "' does not indicate a valid file");
    }
    std::ifstream recording_meta_filestream(recording_meta_file_path, std::ios_base::binary);
    rapidcsv::Document recording_meta_csv_document(recording_meta_filestream);

    lanelet::projection::UtmProjector utm_projector(
                lanelet::Origin({recording_meta_csv_document.GetCell<FP_DATA_TYPE>("latLocation", 0),
                                 recording_meta_csv_document.GetCell<FP_DATA_TYPE>("lonLocation", 0)}), false);
    lanelet::LaneletMapPtr lanelet_map = lanelet::load(input_file_path.string(), utm_projector, nullptr, lanelet::io::Configuration());

    lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
            lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,
                                                                lanelet::Participants::Vehicle);
    lanelet::routing::RoutingGraphUPtr routing_graph = lanelet::routing::RoutingGraph::build(
                *lanelet_map, *traffic_rules);

    geometry::Vec utm_origin(recording_meta_csv_document.GetCell<FP_DATA_TYPE>("xUtmOrigin", 0),
                             recording_meta_csv_document.GetCell<FP_DATA_TYPE>("yUtmOrigin", 0));

    lanelet::LaneletLayer &lanelet_layer = lanelet_map->laneletLayer;

    for (lanelet::Lanelet &current_lanelet : lanelet_layer)
    {
        uint64_t left_adjacent_lane_id;
        lanelet::Optional<lanelet::ConstLanelet> left_adjacent_lanelet =
                routing_graph->left(current_lanelet);
        if (left_adjacent_lanelet)
        {
            left_adjacent_lane_id = left_adjacent_lanelet.value().id();
        }
        else
        {
            left_adjacent_lane_id = 0;
        }

        uint64_t right_adjacent_lane_id;
        lanelet::Optional<lanelet::ConstLanelet> right_adjacent_lanelet =
                routing_graph->right(current_lanelet);
        if (right_adjacent_lanelet)
        {
            right_adjacent_lane_id = right_adjacent_lanelet.value().id();
        }
        else
        {
            right_adjacent_lane_id = 0;
        }

        structures::IStackArray<uint64_t> *fore_lanes =
                new structures::stl::STLStackArray<uint64_t>();
        lanelet::ConstLanelets fore_lanelets = routing_graph->following(current_lanelet, false);
        for (lanelet::ConstLanelet &current_fore_lanelet : fore_lanelets)
        {
            fore_lanes->push_back(current_fore_lanelet.id());
        }

        structures::IStackArray<uint64_t> *aft_lanes =
                new structures::stl::STLStackArray<uint64_t>();
        lanelet::ConstLanelets aft_lanelets = routing_graph->previous(current_lanelet, false);
        for (lanelet::ConstLanelet &current_aft_lanelet : aft_lanelets)
        {
            aft_lanes->push_back(current_aft_lanelet.id());
        }

        id_to_lane_dict.update(current_lanelet.id(), new LaneletDLane(current_lanelet.id(),
                                                                      left_adjacent_lane_id,
                                                                      right_adjacent_lane_id,
                                                                      fore_lanes,
                                                                      aft_lanes,
                                                                      this, current_lanelet,
                                                                      utm_origin));
    }
}

}
}
}
}
