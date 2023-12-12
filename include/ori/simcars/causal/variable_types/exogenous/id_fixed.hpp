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
    uint64_t const value;

public:
    IdFixedVariable(uint64_t value);

    uint64_t get_value() const override;
};

}
}
}
