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

    structures::stl::STLStackArray<uint64_t> get_value() const override;
};

}
}
}
