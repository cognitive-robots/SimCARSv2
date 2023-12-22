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

    IVariable<uint64_t> *parent;

public:
    IdSocketVariable(uint64_t default_value = 0.0, IVariable<uint64_t> *parent = nullptr);

    bool get_value(uint64_t &val) const override;

    IVariable<uint64_t> const* get_parent() const;

    bool set_value(uint64_t const &val) override;

    void set_parent(IVariable<uint64_t> *parent);
};

}
}
}
