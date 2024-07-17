
#include <ori/simcars/map/thor_magni/thor_magni_map.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <fstream>
#include <filesystem>

namespace ori
{
namespace simcars
{
namespace map
{
namespace thor_magni
{

geometry::Vec ThorMagniMap::get_map_centre() const
{
    return geometry::Vec(0, 0);
}

FP_DATA_TYPE ThorMagniMap::get_max_dim_size() const
{
    sf::Vector2u texture_size = texture.getSize();
    return std::max(texture_size.x, texture_size.y);
}

sf::Sprite* ThorMagniMap::get_sprite() const
{
    sf::Sprite *sprite = new sf::Sprite();
    sprite->setTexture(texture, true);
    sprite->move(0.4 * texture_offset.x(), 0.4 * texture_offset.y());
    sprite->scale(0.4, 0.4);
    return sprite;
}

void ThorMagniMap::load(std::string const &texture_file_path_str,
                        std::string const &offset_json_file_path_str,
                        std::string const &scene_file_path_str)
{
    std::filesystem::path texture_file_path(texture_file_path_str);

    std::string texture_filename = texture_file_path.filename();
    if (texture_filename.size() < 4)
    {
        throw std::runtime_error("Invalid texture filename format (less than 4 chars)");
    }
    std::string month_num_str = texture_filename.substr(2, 2);
    size_t month_num = std::atoi(month_num_str.c_str());

    if (!std::filesystem::is_regular_file(texture_file_path))
    {
        throw std::invalid_argument("Texture file path '" + texture_file_path_str +
                                    "' does not indicate a valid file");
    }

    texture.loadFromFile(texture_file_path_str);

    std::filesystem::path offset_json_file_path(offset_json_file_path_str);

    if (!std::filesystem::is_regular_file(offset_json_file_path))
    {
        throw std::invalid_argument("Offset JSON file path '" + texture_file_path_str +
                                    "' does not indicate a valid file");
    }

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

    // TODO: Utilise scene file
}

}
}
}
}
