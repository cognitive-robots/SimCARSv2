#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/valueless_variable_abstract.hpp>

#include <sstream>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class AVariable : public virtual IVariable<T>, public virtual AValuelessVariable
{
public:
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

    structures::IArray<IValuelessEvent const*>* get_valueless_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) const override
    {
        structures::IArray<IEvent<T> const*>* const events =
                this->get_events(time_window_start, time_window_end);
        structures::IArray<IValuelessEvent const*>* const valueless_events =
                    new structures::stl::STLStackArray<IValuelessEvent const*>(events->count());
        cast_array(*events, *valueless_events);
        delete events;
        return valueless_events;
    }
    IValuelessEvent const* get_valueless_event(temporal::Time time, bool exact) const override
    {
        return this->get_event(time, exact);
    }
};

}
}
}
