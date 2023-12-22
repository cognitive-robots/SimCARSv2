#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class IdsPreviousTimeStepVariable :
        public AUnaryEndogenousVariable<structures::stl::STLStackArray<uint64_t>,
        structures::stl::STLStackArray<uint64_t>>
{
public:
    using AUnaryEndogenousVariable<structures::stl::STLStackArray<uint64_t>,
    structures::stl::STLStackArray<uint64_t>>::AUnaryEndogenousVariable;

    bool get_value(structures::stl::STLStackArray<uint64_t> &val) const override;

    bool set_value(structures::stl::STLStackArray<uint64_t> const &val) override;
};

}
}
}
