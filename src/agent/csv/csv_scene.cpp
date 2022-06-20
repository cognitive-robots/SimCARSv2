
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/defines.hpp>
#include <ori/simcars/agent/constant.hpp>
#include <ori/simcars/agent/event.hpp>
#include <ori/simcars/agent/variable.hpp>
#include <ori/simcars/agent/entity.hpp>
#include <ori/simcars/agent/csv/csv_scene.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace csv
{

std::shared_ptr<const CSVScene> CSVScene::construct_from(std::shared_ptr<const IScene> scene)
{
    std::shared_ptr<CSVScene> new_scene(new CSVScene());

    new_scene->min_spatial_limits = scene->get_min_spatial_limits();
    new_scene->max_spatial_limits = scene->get_max_spatial_limits();
    new_scene->min_temporal_limit = scene->get_min_temporal_limit();
    new_scene->max_temporal_limit = scene->get_max_temporal_limit();

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = scene->get_entities();

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<IEntity> new_entity = (*entities)[i];

        new_scene->entity_dict.update(new_entity->get_name(), new_entity);
    }

    return new_scene;
}

void CSVScene::save_virt(std::ofstream &output_filestream) const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> constants = this->get_constants();
    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> variables = this->get_variables();

    size_t i;
    for (i = 0; i < variables->count(); ++i)
    {
        std::string variable_name = (*variables)[i]->get_full_name();
        output_filestream << variable_name << ";";
    }
    for (i = 0; i < constants->count(); ++i)
    {
        std::string constant_name = (*constants)[i]->get_full_name();
        output_filestream << constant_name << ";";
    }
    output_filestream << std::endl;

    temporal::Duration time_step = temporal::Duration(temporal::DurationRep(1000.0 / OPERATING_FRAMERATE));
    temporal::Time current_time;
    for (current_time = min_temporal_limit; current_time <= max_temporal_limit; current_time += time_step)
    {
        for (i = 0; i < variables->count(); ++i)
        {
            try
            {
                std::string value_as_string = (*variables)[i]->get_value_as_string(current_time);
                output_filestream << value_as_string << ";";
            }
            catch (std::out_of_range)
            {
                // Not the best solution, since it's not impossible a variable could take the value "NA"
                output_filestream << "NA;";
            }
        }
        for (i = 0; i < constants->count(); ++i)
        {
            std::string value_as_string = (*constants)[i]->get_value_as_string();
            output_filestream << value_as_string << ";";
        }
        output_filestream << std::endl;
    }
}

void CSVScene::load_virt(std::ifstream& input_filestream)
{
    throw utils::NotImplementedException();
}

geometry::Vec CSVScene::get_min_spatial_limits() const
{
    return this->min_spatial_limits;
}

geometry::Vec CSVScene::get_max_spatial_limits() const
{
    return this->max_spatial_limits;
}

temporal::Time CSVScene::get_min_temporal_limit() const
{
    return this->min_temporal_limit;
}

temporal::Time CSVScene::get_max_temporal_limit() const
{
    return this->max_temporal_limit;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> CSVScene::get_entities() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IEntity>>(entity_dict.get_values()));
}

std::shared_ptr<const IEntity> CSVScene::get_entity(const std::string& entity_name) const
{
    return entity_dict[entity_name];
}

}
}
}
}
