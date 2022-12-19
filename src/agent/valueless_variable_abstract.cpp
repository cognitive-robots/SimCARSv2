
#include <ori/simcars/agent/valueless_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

std::string AValuelessVariable::get_full_name() const
{
    return this->get_entity_name() + "." + this->get_parameter_name();
}

std::string AValuelessVariable::get_parameter_name() const
{
    return this->get_variable_name() + "." + this->get_type_name();
}

std::string AValuelessVariable::get_type_name() const
{
    switch (this->get_type())
    {
        case IValuelessVariable::Type::BASE:
            return "base";

        case IValuelessVariable::Type::INDIRECT_ACTUATION:
            return "indirect_actuation";

        case IValuelessVariable::Type::DIRECT_ACTUATION:
            return "direct_actuation";

        case IValuelessVariable::Type::GOAL:
            return "goal";

        case IValuelessVariable::Type::EXTERNAL:
            return "external";

        default:
            throw std::runtime_error("Type indicated by value: '" + std::to_string((uint8_t) this->get_type()) + "' is not supported");
    };
}

}
}
}
