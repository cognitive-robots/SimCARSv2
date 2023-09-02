#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarSocketVariable : public IExogenousVariable<FP_DATA_TYPE>
{
    FP_DATA_TYPE default_value;

    IVariable<FP_DATA_TYPE> const *parent;

public:
    ScalarSocketVariable(FP_DATA_TYPE default_value = 0.0,
                         IVariable<FP_DATA_TYPE> const *parent = nullptr);

    FP_DATA_TYPE get_value() const override;

    IVariable<FP_DATA_TYPE> const* get_parent() const;

    void set_parent(IVariable<FP_DATA_TYPE> const *parent);
};

}
}
}
