#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/temporal_rounding_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorBufferVariable : public AUnaryEndogenousVariable<geometry::Vec, geometry::Vec>
{
    temporal::TemporalRoundingDictionary<geometry::Vec>* temporal_dictionary;

    bool axiomatic;

public:
    VectorBufferVariable(
            IVariable<geometry::Vec> const *parent,
            temporal::TemporalRoundingDictionary<geometry::Vec> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~VectorBufferVariable();

    geometry::Vec get_value() const override;
};

}
}
}
