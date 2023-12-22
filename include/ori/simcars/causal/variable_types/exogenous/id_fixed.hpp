#pragma once

#include <ori/simcars/causal/exogenous_variable_interface.hpp>

#include <cstdint>

namespace ori
{
namespace simcars
{
namespace causal
{

class IdFixedVariable : public IExogenousVariable<uint64_t>
{
    uint64_t value;

public:
    IdFixedVariable(uint64_t value);

    bool get_value(uint64_t &val) const override;

    bool set_value(uint64_t const &val) override;
};

}
}
}
