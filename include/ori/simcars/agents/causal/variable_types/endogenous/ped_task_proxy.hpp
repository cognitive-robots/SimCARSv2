#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/ped_task.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedTaskProxyVariable : public simcars::causal::AUnaryEndogenousVariable<PedTask, PedTask>
{
public:
    using AUnaryEndogenousVariable<PedTask, PedTask>::AUnaryEndogenousVariable;

    bool get_value(PedTask &val) const override;

    bool set_value(PedTask const &val) override;
};

}
}
}
}
