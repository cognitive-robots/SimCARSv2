#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarBufferVariable : public AUnaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>
{
    temporal::TemporalDictionary<FP_DATA_TYPE>* temporal_dictionary;

    bool axiomatic;

public:
    ScalarBufferVariable(
            IVariable<FP_DATA_TYPE> *parent,
            temporal::TemporalDictionary<FP_DATA_TYPE> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~ScalarBufferVariable();

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;

    void set_axiomatic(bool axiomatic);
};

}
}
}
