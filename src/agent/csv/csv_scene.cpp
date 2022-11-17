
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/defines.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/agent/basic_variable.hpp>
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

CSVScene const* CSVScene::construct_from(IScene const *scene)
{
    CSVScene *new_scene = new CSVScene();

    new_scene->min_spatial_limits = scene->get_min_spatial_limits();
    new_scene->max_spatial_limits = scene->get_max_spatial_limits();
    new_scene->min_temporal_limit = scene->get_min_temporal_limit();
    new_scene->max_temporal_limit = scene->get_max_temporal_limit();

    structures::IArray<IEntity const*> *entities = scene->get_entities();

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        new_scene->entity_dict.update((*entities)[i]->get_name(), (*entities)[i]);
    }

    return new_scene;
}

void CSVScene::save_virt(std::ofstream &output_filestream) const
{
    structures::IArray<IValuelessConstant const*> *constants = this->get_constants();
    structures::IArray<IValuelessVariable const*> *variables = this->get_variables();

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

void CSVScene::load_virt(std::ifstream &input_filestream)
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

structures::IArray<IEntity const*>* CSVScene::get_entities() const
{
    structures::stl::STLStackArray<IEntity const*> *entities =
            new structures::stl::STLStackArray<IEntity const*>(entity_dict.count());
    entity_dict.get_values(entities);
    return entities;
}

IEntity const* CSVScene::get_entity(std::string const &entity_name) const
{
    return entity_dict[entity_name];
}

IState* CSVScene::get_state(temporal::Time time) const
{
    throw utils::NotImplementedException();
}

}
}
}
}
