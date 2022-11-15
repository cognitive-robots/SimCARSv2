#pragma once

#include <exception>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T>
class ISoul
{
public:
    class GhostObjectException : public std::exception {
    public:
        using std::exception::exception;

        char const* what() const noexcept override
        {
            return "Object is a ghost";
        }
    };

    virtual ~ISoul() = default;

    virtual bool is_ghost() const = 0;

    virtual T const* get_self() const = 0;

    virtual T const* get_true_self() const noexcept = 0;
};

}
}
}
