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
    FP_DATA_TYPE value;

public:
    ScalarFixedVariable(FP_DATA_TYPE value);

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
