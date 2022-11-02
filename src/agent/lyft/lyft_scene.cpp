
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>

#include <lz4_stream.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

std::shared_ptr<const LyftScene> LyftScene::construct_from(std::shared_ptr<const IDrivingScene> driving_scene)
{
    std::shared_ptr<LyftScene> new_driving_scene(new LyftScene());

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = driving_scene->get_max_temporal_limit();

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> driving_agents =
            driving_scene->get_driving_agents();

    size_t i;
    for(i = 0; i < driving_agents->count(); ++i)
    {
        new_driving_scene->driving_agent_dict.update((*driving_agents)[i]->get_name(), (*driving_agents)[i]);
    }

    return new_driving_scene;
}

void LyftScene::save_virt(std::ofstream &output_filestream) const
{
    throw utils::NotImplementedException();
}

void LyftScene::load_virt(std::ifstream& input_filestream)
{
    lz4_stream::istream input_lz4_stream(input_filestream);
    rapidjson::BasicIStreamWrapper input_lz4_json_stream(input_lz4_stream);

    rapidjson::Document json_document;
    json_document.ParseStream(input_lz4_json_stream);

    this->min_temporal_limit = temporal::Time::max();
    this->max_temporal_limit = temporal::Time::min();

    FP_DATA_TYPE min_position_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_position_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_y = std::numeric_limits<FP_DATA_TYPE>::min();

    for (const rapidjson::Value& json_document_element : json_document.GetArray())
    {
        const rapidjson::Value::ConstObject& json_agent_data = json_document_element.GetObject();

        std::shared_ptr<const LyftDrivingAgent> driving_agent(new LyftDrivingAgent(json_agent_data));

        driving_agent_dict.update(driving_agent->get_name(), driving_agent);

        this->min_temporal_limit = std::min(driving_agent->get_min_temporal_limit(), this->min_temporal_limit);
        this->max_temporal_limit = std::max(driving_agent->get_max_temporal_limit(), this->max_temporal_limit);

        geometry::Vec driving_agent_min_spatial_limits = driving_agent->get_min_spatial_limits();
        min_position_x = std::min(driving_agent_min_spatial_limits.x(), min_position_x);
        min_position_y = std::min(driving_agent_min_spatial_limits.y(), min_position_y);

        geometry::Vec driving_agent_max_spatial_limits = driving_agent->get_max_spatial_limits();
        max_position_x = std::max(driving_agent_max_spatial_limits.x(), max_position_x);
        max_position_y = std::max(driving_agent_max_spatial_limits.y(), max_position_y);
    }

    this->min_spatial_limits = geometry::Vec(min_position_x, min_position_y);
    this->max_spatial_limits = geometry::Vec(max_position_x, max_position_y);
}

geometry::Vec LyftScene::get_min_spatial_limits() const
{
    return this->min_spatial_limits;
}

geometry::Vec LyftScene::get_max_spatial_limits() const
{
    return this->max_spatial_limits;
}

temporal::Time LyftScene::get_min_temporal_limit() const
{
    return this->min_temporal_limit;
}

temporal::Time LyftScene::get_max_temporal_limit() const
{
    return this->max_temporal_limit;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> LyftScene::get_entities() const
{
    const std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> driving_agents = this->get_driving_agents();
    const std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities(
                new structures::stl::STLStackArray<std::shared_ptr<const IEntity>>(driving_agents->count()));
    cast_array(*driving_agents, *entities);
    return entities;
}

std::shared_ptr<const IEntity> LyftScene::get_entity(const std::string& entity_name) const
{
    return this->get_driving_agent(entity_name);
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> LyftScene::get_driving_agents() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>>(
                new structures::stl::STLStackArray<std::shared_ptr<const IDrivingAgent>>(
                    driving_agent_dict.get_values()));
}

std::shared_ptr<const IDrivingAgent> LyftScene::get_driving_agent(const std::string& driving_agent_name) const
{
    return driving_agent_dict[driving_agent_name];
}

}
}
}
}
