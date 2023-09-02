#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

#include <cstdint>

namespace ori
{
namespace simcars
{
namespace causal
{

class IdProxyVariable : public AUnaryEndogenousVariable<uint64_t, uint64_t>
{
public:
    using AUnaryEndogenousVariable<uint64_t, uint64_t>::AUnaryEndogenousVariable;

    uint64_t get_value() const override;
};

}
}
}
