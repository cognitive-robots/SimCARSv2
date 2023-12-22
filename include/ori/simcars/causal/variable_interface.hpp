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

    virtual bool get_value(R &val) const = 0;

    /*
     * WARNING: This is currently only meant for serial assignment of variables. Random assignment
     * of variables once a time series has been established will cause this function to return
     * false.
     */
    virtual bool set_value(R const &val) = 0;
};

}
}
}
