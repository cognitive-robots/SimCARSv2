
#include <ori/simcars/map/thor_magni/thor_magni_map.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <rapidcsv.h>

#include <fstream>
#include <filesystem>
#include <random>

namespace ori
{
namespace simcars
{
namespace map
{
namespace thor_magni
{

bool ThorMagniMap::is_node_within_dist(geometry::Vec const &position, FP_DATA_TYPE dist) const
{
    structures::IArray<ThorMagniNode*> const *node_array = id_to_node_dict.get_values();
    for (size_t i = 0; i < node_array->count(); ++i)
    {
        geometry::Vec const &node_centroid = (*node_array)[i]->get_centroid();
        if ((position - node_centroid).norm() <= dist)
        {
            return true;
        }
    }
    return false;
}

structures::IArray<ThorMagniNode*>* ThorMagniMap::get_nodes_within_dist(
        geometry::Vec const &position, FP_DATA_TYPE dist) const
{
    structures::IStackArray<ThorMagniNode*> *nodes_within_dist =
            new structures::stl::STLStackArray<ThorMagniNode*>;

    structures::IArray<ThorMagniNode*> const *node_array = id_to_node_dict.get_values();
    for (size_t i = 0; i < node_array->count(); ++i)
    {
        geometry::Vec const &node_centroid = (*node_array)[i]->get_centroid();
        if ((position - node_centroid).norm() <= dist)
        {
            nodes_within_dist->push_back((*node_array)[i]);
        }
    }

    return nodes_within_dist;
}

ThorMagniMap::~ThorMagniMap()
{
    structures::IArray<ThorMagniNode*> const *nodes = id_to_node_dict.get_values();
    for (size_t i = 0; i < nodes->count(); ++i)
    {
        delete (*nodes)[i];
    }
}

geometry::Vec ThorMagniMap::get_map_centre() const
{
    sf::Vector2u texture_size = texture.getSize();
    return get_texture_scale() * (geometry::Vec(texture_offset.x(), -texture_offset.y()) +
                                  geometry::Vec(texture_size.x / 2.0, texture_size.y / -2.0));
}

geometry::Rect ThorMagniMap::get_bounding_box() const
{
    structures::IArray<ThorMagniNode*> const *node_array = id_to_node_dict.get_values();

    FP_DATA_TYPE min_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_y = std::numeric_limits<FP_DATA_TYPE>::min();

    for (size_t i = 0; i < node_array->count(); ++i)
    {
        geometry::Vec const &centroid = (*node_array)[i]->get_centroid();
        min_x = std::min(centroid.x(), min_x);
        max_x = std::max(centroid.x(), max_x);
        min_y = std::min(centroid.y(), min_y);
        max_y = std::max(centroid.y(), max_y);
    }

    return geometry::Rect(min_x, min_y, max_x, max_y);
}

FP_DATA_TYPE ThorMagniMap::get_max_dim_size() const
{
    sf::Vector2u texture_size = texture.getSize();
    return get_texture_scale() * std::max(texture_size.x, texture_size.y);
}

sf::Texture const& ThorMagniMap::get_texture() const
{
    return texture;
}

FP_DATA_TYPE ThorMagniMap::get_texture_scale() const
{
    return 0.01;
}

geometry::Vec const& ThorMagniMap::get_texture_offset() const
{
    return texture_offset;
}

structures::IArray<INode const*>* ThorMagniMap::get_nodes() const
{
    structures::IArray<INode const*> *node_array =
            new structures::stl::STLStackArray<INode const*>(id_to_node_dict.count());
    cast_array<ThorMagniNode*, INode const*>(*(id_to_node_dict.get_values()), *node_array);
    return node_array;
}

structures::IArray<INode const*>* ThorMagniMap::get_goal_nodes() const
{
    structures::IStackArray<INode const*> *goal_node_array =
            new structures::stl::STLStackArray<INode const*>;

    for (uint64_t i = 0; i <= max_goal_id; ++i)
    {
        if (id_to_node_dict.contains(i))
        {
            goal_node_array->push_back(id_to_node_dict[i]);
        }
    }

    return goal_node_array;
}

structures::IArray<geometry::VecPair>* ThorMagniMap::get_edges() const
{
    structures::IStackArray<geometry::VecPair> *edge_array =
            new structures::stl::STLStackArray<geometry::VecPair>;

    structures::IArray<ThorMagniNode*> const *node_array = id_to_node_dict.get_values();

    size_t i, j;
    for (i = 0; i < node_array->count(); ++i)
    {
        INode const *node = (*node_array)[i];
        structures::IArray<INode const*> const *adjacent = node->get_adjacent();
        for (j = 0; j < adjacent->count(); ++j)
        {
            edge_array->push_back(geometry::VecPair(node->get_centroid(),
                                                    (*adjacent)[j]->get_centroid()));
        }
    }

    return edge_array;
}

void ThorMagniMap::load(std::string const &texture_file_path_str,
                        std::string const &offset_json_file_path_str,
                        std::string const &scene_file_path_str,
                        std::string const &goals_file_path_str, FP_DATA_TYPE node_clearance,
                        FP_DATA_TYPE node_adjacency, size_t random_sample_count)
{
    std::filesystem::path texture_file_path(texture_file_path_str);

    std::string texture_filename = texture_file_path.filename();
    if (texture_filename.size() < 4)
    {
        throw std::runtime_error("Invalid texture filename format (less than 4 chars)");
    }
    std::string date_str = texture_filename.substr(0, 4);
    std::string month_num_str = texture_filename.substr(2, 2);
    size_t month_num = std::atoi(month_num_str.c_str());

    if (!std::filesystem::is_regular_file(texture_file_path))
    {
        throw std::invalid_argument("Texture file path '" + texture_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path offset_json_file_path(offset_json_file_path_str);

    if (!std::filesystem::is_regular_file(offset_json_file_path))
    {
        throw std::invalid_argument("Offset JSON file path '" + offset_json_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path scene_file_path(scene_file_path_str);

    if (!std::filesystem::is_regular_file(scene_file_path))
    {
        throw std::invalid_argument("Scene file path '" + scene_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path goals_file_path(goals_file_path_str);

    if (!std::filesystem::is_regular_file(goals_file_path))
    {
        throw std::invalid_argument("Goals file path '" + goals_file_path_str +
                                    "' does not indicate a valid file");
    }


    image.loadFromFile(texture_file_path_str);
    texture.loadFromImage(image);


    std::ifstream offset_json_filestream(offset_json_file_path_str);
    rapidjson::IStreamWrapper offset_json_stream(offset_json_filestream);

    rapidjson::Document offset_json_document;
    offset_json_document.ParseStream(offset_json_stream);

    rapidjson::Value offset_json;

    switch (month_num)
    {
    case 1:
        offset_json = offset_json_document["January"];
        break;
    case 2:
        offset_json = offset_json_document["February"];
        break;
    case 3:
        offset_json = offset_json_document["March"];
        break;
    case 4:
        offset_json = offset_json_document["April"];
        break;
    case 5:
        offset_json = offset_json_document["May"];
        break;
    case 6:
        offset_json = offset_json_document["June"];
        break;
    case 7:
        offset_json = offset_json_document["July"];
        break;
    case 8:
        offset_json = offset_json_document["August"];
        break;
    case 9:
        offset_json = offset_json_document["September"];
        break;
    case 10:
        offset_json = offset_json_document["October"];
        break;
    case 11:
        offset_json = offset_json_document["November"];
        break;
    case 12:
        offset_json = offset_json_document["December"];
        break;
    default:
        throw std::runtime_error("Invalid month specified by texture filename");
    }

    sf::Vector2u texture_size = texture.getSize();
    texture_offset = geometry::Vec(offset_json[0].GetInt(),
            -int16_t(texture_size.y) - offset_json[1].GetInt());


    rapidcsv::Document goals_doc(goals_file_path_str);

    max_goal_id = 0;

    size_t i, j;
    for (i = 0; i < goals_doc.GetRowCount(); ++i)
    {
        if (goals_doc.GetCell<std::string>("day", i) == date_str)
        {
            uint64_t id = goals_doc.GetCell<uint64_t>("goal", i);
            max_goal_id = std::max(id, max_goal_id);

            FP_DATA_TYPE x = goals_doc.GetCell<FP_DATA_TYPE>("x", i) / 1000.0;
            FP_DATA_TYPE y = goals_doc.GetCell<FP_DATA_TYPE>("y", i) / 1000.0;

            ThorMagniNode *node = new ThorMagniNode(id, this, geometry::Vec(x, y));
            id_to_node_dict.update(id, node);
        }
    }

    uint64_t id = max_goal_id + 1;

    rapidcsv::Document scene_doc(scene_file_path_str);

    for (i = 0; i < scene_doc.GetRowCount(); ++i)
    {
        std::string const position_x_str = scene_doc.GetCell<std::string>("x", i);
        std::string const position_y_str = scene_doc.GetCell<std::string>("y", i);

        if (position_x_str != "" && position_y_str != "")
        {
            FP_DATA_TYPE const position_x = std::strtod(position_x_str.c_str(), nullptr) / 1000.0;
            FP_DATA_TYPE const position_y = std::strtod(position_y_str.c_str(), nullptr) / 1000.0;
            geometry::Vec const position(position_x, position_y);

            geometry::Vec const texture_position = (geometry::Vec(position_x, -position_y) /
                                                    get_texture_scale()) - get_texture_offset();

            if (image.getPixel(texture_position.x(), texture_position.y()) == sf::Color::White &&
                    !is_node_within_dist(position, node_clearance))
            {
                structures::IArray<ThorMagniNode*> *adjacent =
                        get_nodes_within_dist(position, node_adjacency);

                ThorMagniNode *node = new ThorMagniNode(id, this, position);
                id_to_node_dict.update(id, node);
                ++id;

                for (j = 0; j < adjacent->count(); ++j)
                {
                    geometry::Vec const mid_point =
                            0.5 * (position + (*adjacent)[j]->get_centroid());

                    geometry::Vec const mid_point_texture_position =
                            (geometry::Vec(mid_point.x(), -mid_point.y()) /
                             get_texture_scale()) - get_texture_offset();

                    // TODO: Improve edge collision detection
                    if (image.getPixel(mid_point_texture_position.x(),
                                       mid_point_texture_position.y()) == sf::Color::White)
                    {
                        node->add_adjacent((*adjacent)[j]);
                        (*adjacent)[j]->add_adjacent(node);
                    }
                }

                delete adjacent;
            }
        }
    }

    geometry::Rect bounding_box = get_bounding_box();

    std::default_random_engine random_engine;
    std::uniform_real_distribution<FP_DATA_TYPE> x_distribution(bounding_box.get_min_x(),
                                                                bounding_box.get_max_x());
    std::uniform_real_distribution<FP_DATA_TYPE> y_distribution(bounding_box.get_min_y(),
                                                                bounding_box.get_max_y());

    for (i = 0; i < random_sample_count; ++i)
    {
        FP_DATA_TYPE const position_x = x_distribution(random_engine);
        FP_DATA_TYPE const position_y = y_distribution(random_engine);
        geometry::Vec const position(position_x, position_y);

        geometry::Vec const texture_position = (geometry::Vec(position_x, -position_y) /
                                                get_texture_scale()) - get_texture_offset();

        if (image.getPixel(texture_position.x(), texture_position.y()) == sf::Color::White &&
                !is_node_within_dist(position, node_clearance))
        {
            structures::IArray<ThorMagniNode*> *adjacent =
                    get_nodes_within_dist(position, node_adjacency);

            ThorMagniNode *node = new ThorMagniNode(id, this, position);
            id_to_node_dict.update(id, node);
            ++id;

            for (j = 0; j < adjacent->count(); ++j)
            {
                geometry::Vec const mid_point = 0.5 * (position + (*adjacent)[j]->get_centroid());

                geometry::Vec const mid_point_texture_position =
                        (geometry::Vec(mid_point.x(), -mid_point.y()) /
                         get_texture_scale()) - get_texture_offset();

                // TODO: Improve edge collision detection
                if (image.getPixel(mid_point_texture_position.x(),
                                   mid_point_texture_position.y()) == sf::Color::White)
                {
                    node->add_adjacent((*adjacent)[j]);
                    (*adjacent)[j]->add_adjacent(node);
                }
            }

            delete adjacent;
        }
    }
}

}
}
}
}
