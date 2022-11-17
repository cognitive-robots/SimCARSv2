
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

structures::IArray<IValuelessConstant const*>* AScene::get_constants() const
{
    structures::IArray<IEntity const*> *entities = this->get_entities();

    structures::stl::STLConcatArray<IValuelessConstant const*> *constants =
            new structures::stl::STLConcatArray<IValuelessConstant const*>(entities->count());

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        constants->get_array(i) = (*entities)[i]->get_constant_parameters();
    }

    return constants;
}

IValuelessConstant const* AScene::get_constant(std::string const &constant_name) const
{
    return this->get_entity(constant_name.substr(0, constant_name.find(".")))->get_constant_parameter(constant_name);
}

structures::IArray<IValuelessVariable const*>* AScene::get_variables() const
{
    structures::IArray<IEntity const*> *entities = this->get_entities();

    structures::stl::STLConcatArray<IValuelessVariable const*> *variables =
            new structures::stl::STLConcatArray<IValuelessVariable const*>(entities->count());

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        variables->get_array(i) = (*entities)[i]->get_variable_parameters();
    }

    return variables;
}

IValuelessVariable const* AScene::get_variable(std::string const &variable_name) const
{
    return this->get_entity(variable_name.substr(0, variable_name.find(".")))->get_variable_parameter(variable_name);
}

structures::IArray<IValuelessEvent const*>* AScene::get_events() const
{
    structures::IArray<IEntity const*> *entities = this->get_entities();

    structures::stl::STLConcatArray<IValuelessEvent const*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent const*>(entities->count());

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        events->get_array(i) = (*entities)[i]->get_events();
    }

    return events;
}

}
}
}
