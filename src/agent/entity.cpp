
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/entity.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

Entity::Entity(const std::string& entity_name) : name(entity_name) {}

std::shared_ptr<IEntity> Entity::deep_copy() const
{
    std::shared_ptr<Entity> entity(new Entity(name));

    size_t i;

    std::shared_ptr<const structures::IArray<std::string>> constant_names = constant_dict.get_keys();
    for(i = 0; i < constant_names->count(); ++i)
    {
        entity->constant_dict.update((*constant_names)[i], constant_dict[(*constant_names)[i]]->valueless_shallow_copy());
    }

    std::shared_ptr<const structures::IArray<std::string>> variable_names = variable_dict.get_keys();
    for(i = 0; i < variable_names->count(); ++i)
    {
        entity->variable_dict.update((*variable_names)[i], variable_dict[(*variable_names)[i]]->valueless_deep_copy());
    }

    return entity;
}

std::string Entity::get_name() const
{
    return name;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> Entity::get_constant_parameters() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IValuelessConstant>>(constant_dict.get_values()));
}

std::shared_ptr<const IValuelessConstant> Entity::get_constant_parameter(const std::string& constant_name) const
{
    return constant_dict[constant_name];
}

bool Entity::add_constant_parameter(const std::string& constant_name, std::shared_ptr<const IValuelessConstant> valueless_constant)
{
    if (constant_dict.contains(constant_name))
    {
        return false;
    }
    else
    {
        constant_dict.update(constant_name, valueless_constant);
        return true;
    }
}

bool Entity::remove_constant_parameter(const std::string& constant_name)
{
    if (constant_dict.contains(constant_name))
    {
        constant_dict.erase(constant_name);
        return true;
    }
    else
    {
        return false;
    }
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> Entity::get_variable_parameters() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IValuelessVariable>>(variable_dict.get_values()));
}

std::shared_ptr<const IValuelessVariable> Entity::get_variable_parameter(const std::string& variable_name) const
{
    return variable_dict[variable_name];
}

bool Entity::add_variable_parameter(const std::string& variable_name, std::shared_ptr<const IValuelessVariable> valueless_variable)
{
    if (variable_dict.contains(variable_name))
    {
        return false;
    }
    else
    {
        variable_dict.update(variable_name, valueless_variable);
        return true;
    }
}

bool Entity::remove_variable_parameter(const std::string& variable_name)
{
    if (variable_dict.contains(variable_name))
    {
        variable_dict.erase(variable_name);
        return true;
    }
    else
    {
        return false;
    }
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> Entity::get_events() const
{
    std::shared_ptr<const structures::IArray<std::string>> variable_names = variable_dict.get_keys();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>> events(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>(variable_names->count()));

    size_t i;
    for(i = 0; i < variable_names->count(); ++i)
    {
        events->get_array(i) = variable_dict[(*variable_names)[i]]->get_valueless_events();
    }

    return events;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> Entity::get_events(temporal::Time time) const
{
    std::shared_ptr<const structures::IArray<std::string>> variable_names = variable_dict.get_keys();

    std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const IValuelessEvent>>> events;

    std::shared_ptr<const IValuelessEvent> event;

    size_t i;
    for(i = 0; i < variable_names->count(); ++i)
    {
        try
        {
            events->push_back(variable_dict[(*variable_names)[i]]->get_valueless_event(time));
        }
        catch (std::out_of_range)
        {
            // This variable did not have an event at this specific time
        }
    }

    return events;
}

}
}
}
