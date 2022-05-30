#pragma once

#include <ori/simcars/agent/event_interface.hpp>

#include <sstream>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class AEvent : public IEvent<T>
{
protected:
    bool is_equal(std::shared_ptr<const IValuelessEvent> event) const override
    {
        std::shared_ptr<const AEvent<T>> cast_event = std::static_pointer_cast<const AEvent<T>>(event);
        return this->get_variable_name() == cast_event->get_variable_name() &&
                this->get_value() == cast_event->get_value() &&
                this->get_time() == cast_event->get_time();
    }

public:
    std::string get_value_as_string() const override
    {
        std::stringstream string_stream;
        string_stream << this->get_value();
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
};

}
}
}
