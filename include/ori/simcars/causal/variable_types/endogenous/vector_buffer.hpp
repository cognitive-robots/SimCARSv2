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
    temporal::TemporalRoundingDictionary<geometry::Vec>* const temporal_dictionary;

public:
    VectorBufferVariable(IVariable<geometry::Vec> const *parent);
    VectorBufferVariable(IVariable<geometry::Vec> const *parent,
                         temporal::TemporalRoundingDictionary<geometry::Vec> *temporal_dictionary);

    ~VectorBufferVariable();

    geometry::Vec get_value() const override;
};

}
}
}
