
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> AScene::get_constants() const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = this->get_entities();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessConstant>>> constants(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessConstant>>(entities->count()));

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        constants->get_array(i) = (*entities)[i]->get_constant_parameters();
    }

    return constants;
}

std::shared_ptr<const IValuelessConstant> AScene::get_constant(const std::string& constant_name) const
{
    return this->get_entity(constant_name.substr(0, constant_name.find(".")))->get_constant_parameter(constant_name);
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> AScene::get_variables() const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = this->get_entities();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessVariable>>> variables(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessVariable>>(entities->count()));

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        variables->get_array(i) = (*entities)[i]->get_variable_parameters();
    }

    return variables;
}

std::shared_ptr<const IValuelessVariable> AScene::get_variable(const std::string& variable_name) const
{
    return this->get_entity(variable_name.substr(0, variable_name.find(".")))->get_variable_parameter(variable_name);
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> AScene::get_events() const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = this->get_entities();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>> events(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>(entities->count()));

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        events->get_array(i) = (*entities)[i]->get_events();
    }

    return events;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> AScene::get_events(temporal::Time time) const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = this->get_entities();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>> events(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>(entities->count()));

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        events->get_array(i) = (*entities)[i]->get_events(time);
    }

    return events;
}

}
}
}
