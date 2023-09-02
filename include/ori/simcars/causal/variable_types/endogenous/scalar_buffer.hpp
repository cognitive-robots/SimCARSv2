#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/temporal_rounding_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarBufferVariable : public AUnaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>
{
    temporal::TemporalRoundingDictionary<FP_DATA_TYPE>* const temporal_dictionary;

public:
    ScalarBufferVariable(IVariable<FP_DATA_TYPE> const *parent);
    ScalarBufferVariable(IVariable<FP_DATA_TYPE> const *parent,
                         temporal::TemporalRoundingDictionary<FP_DATA_TYPE> *temporal_dictionary);

    ~ScalarBufferVariable();

    FP_DATA_TYPE get_value() const override;
};

}
}
}
