#pragma once

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename R>
class IVariable
{
public:
    virtual ~IVariable() = default;

    virtual R get_value() const = 0;
};

}
}
}
