#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/ped_task.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedTaskSocketVariable : public simcars::causal::IExogenousVariable<PedTask>
{
    PedTask default_value;

    IVariable<PedTask> *parent;

public:
    PedTaskSocketVariable(PedTask default_value = PedTask(),
                          IVariable<PedTask> *parent = nullptr);

    bool get_value(PedTask &val) const override;

    IVariable<PedTask> const* get_parent() const;

    bool set_value(PedTask const &val) override;

    void set_parent(IVariable<PedTask> *parent);
};

}
}
}
}
