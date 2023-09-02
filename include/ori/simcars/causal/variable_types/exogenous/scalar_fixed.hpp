#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarFixedVariable : public IExogenousVariable<FP_DATA_TYPE>
{
    FP_DATA_TYPE const value;

public:
    ScalarFixedVariable(FP_DATA_TYPE value);

    FP_DATA_TYPE get_value() const override;
};

}
}
}
