#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/variable_interface.hpp>

#include <sstream>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class AVariable : public virtual IVariable<T>
{
public:
    std::string get_full_name() const override
    {
        return this->get_entity_name() + "." + this->get_parameter_name() + "." + this->get_type_name();
    }

    std::string get_type_name() const override
    {
        switch (this->get_type())
        {
            case IValuelessVariable::Type::BASE:
                return "base";

            case IValuelessVariable::Type::INDIRECT_ACTUATION:
                return "indirect_actuation";

            case IValuelessVariable::Type::DIRECT_ACTUATION:
                return "direct_actuation";

            case IValuelessVariable::Type::GOAL_VALUE:
                return "goal_value";

            case IValuelessVariable::Type::GOAL_DURATION:
                return "goal_duration";

            case IValuelessVariable::Type::EXTERNAL:
                return "external";

            default:
                throw std::runtime_error("Type indicated by value: '" + std::to_string((uint8_t) this->get_type()) + "' is not supported");
        };
    }

    std::string get_value_as_string(temporal::Time time) const override
    {
        std::stringstream string_stream;
        string_stream << this->get_value(time);
        std::string value_as_string = string_stream.str();

        std::stringstream updated_string_stream;

        size_t i = 0, j;
        while (true)
        {
            j = value_as_string.find('\n', i);

            if (j == i || (j == i + 1 && value_as_string[i] == ','))
            {
                i = j + 1;
                continue;
            }

            std::string line;
            if (j != std::string::npos && value_as_string[j - 1] == ',')
            {
                line = value_as_string.substr(i, (j - i) - 1);
            }
            else
            {
                line = value_as_string.substr(i, j - i);
            }

            updated_string_stream << line;

            if (j == std::string::npos)
            {
                break;
            }
            else
            {
                updated_string_stream << " ";
            }

            i = j + 1;
        }

        return updated_string_stream.str();
    }

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_valueless_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) const override
    {
        const std::shared_ptr<structures::IArray<std::shared_ptr<const IEvent<T>>>> events = this->get_events(
                    time_window_start,
                    time_window_end);
        const std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> valueless_events(
                    new structures::stl::STLStackArray<std::shared_ptr<const IValuelessEvent>>(events->count()));
        cast_array(*events, *valueless_events);
        return valueless_events;
    }

    std::shared_ptr<const IValuelessEvent> get_valueless_event(temporal::Time time) const override
    {
        return this->get_event(time);
    }
};

}
}
}
