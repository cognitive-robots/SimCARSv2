#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarAbsoluteVariable : public AUnaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using AUnaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>::AUnaryEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
