#pragma once

#include <ori/simcars/causal/exogenous_variable_interface.hpp>

#include <cstdint>

namespace ori
{
namespace simcars
{
namespace causal
{

class IdSocketVariable : public IExogenousVariable<uint64_t>
{
    uint64_t default_value;

    IVariable<uint64_t> const *parent;

public:
    IdSocketVariable(uint64_t default_value = 0.0, IVariable<uint64_t> const *parent = nullptr);

    uint64_t get_value() const override;

    IVariable<uint64_t> const* get_parent() const;

    void set_parent(IVariable<uint64_t> const *parent);
};

}
}
}
