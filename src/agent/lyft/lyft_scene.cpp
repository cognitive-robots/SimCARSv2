
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

LyftScene const* LyftScene::construct_from(IDrivingScene const *driving_scene)
{
    LyftScene *new_driving_scene = new LyftScene();

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = driving_scene->get_max_temporal_limit();

    structures::IArray<IDrivingAgent const*> *driving_agents = driving_scene->get_driving_agents();

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

void LyftScene::load_virt(std::ifstream &input_filestream)
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

    for (rapidjson::Value const &json_document_element : json_document.GetArray())
    {
        rapidjson::Value::ConstObject const &json_agent_data = json_document_element.GetObject();

        LyftDrivingAgent const *driving_agent = new LyftDrivingAgent(json_agent_data);

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

structures::IArray<IEntity const*>* LyftScene::get_entities() const
{
    structures::IArray<IDrivingAgent const*>* const driving_agents = this->get_driving_agents();
    structures::IArray<IEntity const*>* const entities =
            new structures::stl::STLStackArray<IEntity const*>(driving_agents->count());
    cast_array(*driving_agents, *entities);
    return entities;
}

IEntity const* LyftScene::get_entity(std::string const &entity_name) const
{
    return this->get_driving_agent(entity_name);
}

structures::IArray<IDrivingAgent const*>* LyftScene::get_driving_agents() const
{
    structures::stl::STLStackArray<IDrivingAgent const*> *driving_agents =
            new structures::stl::STLStackArray<IDrivingAgent const*>(driving_agent_dict.count());
    driving_agent_dict.get_values(driving_agents);
    return driving_agents;
}

IDrivingAgent const* LyftScene::get_driving_agent(std::string const &driving_agent_name) const
{
    return driving_agent_dict[driving_agent_name];
}

}
}
}
}
