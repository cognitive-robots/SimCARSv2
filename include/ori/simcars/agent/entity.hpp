#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/entity_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class Entity : public IEntity
{
    const std::string name;

    structures::stl::STLDictionary<std::string, std::shared_ptr<const IValuelessConstant>> constant_dict;
    structures::stl::STLDictionary<std::string, std::shared_ptr<const IValuelessVariable>> variable_dict;

public:
    Entity(const std::string& entity_name);

    std::shared_ptr<IEntity> deep_copy() const override;

    std::string get_name() const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constant_parameters() const override;
    std::shared_ptr<const IValuelessConstant> get_constant_parameter(const std::string& constant_name) const override;
    bool add_constant_parameter(const std::string& constant_name, std::shared_ptr<const IValuelessConstant> valueless_constant) override;
    bool remove_constant_parameter(const std::string& constant_name) override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variable_parameters() const override;
    std::shared_ptr<const IValuelessVariable> get_variable_parameter(const std::string& variable_name) const override;
    bool add_variable_parameter(const std::string& variable_name, std::shared_ptr<const IValuelessVariable> valueless_variable) override;
    bool remove_variable_parameter(const std::string& variable_name) override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const override;
    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events(temporal::Time time) const override;
};

}
}
}
