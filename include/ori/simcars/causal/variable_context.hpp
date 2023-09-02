#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VariableContext
{
    static thread_local temporal::Time current_time;
    static thread_local temporal::Duration time_step_size;

    VariableContext() = delete;
    VariableContext(VariableContext const&) = delete;
    VariableContext(VariableContext&&) = delete;

public:
    static temporal::Time get_current_time();
    static bool set_current_time(temporal::Time current_time);

    static temporal::Duration get_time_step_size();
    static bool set_time_step_size(temporal::Duration time_step_size);

    static void increment_time_step();
    static void decrement_time_step();
};

}
}
}
