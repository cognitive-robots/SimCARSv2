#pragma once

#include <exception>
#include <memory>

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

        const char* what() const noexcept override
        {
            return "Object is a ghost";
        }
    };

    virtual ~ISoul() = default;

    virtual std::shared_ptr<const T> get_self() const = 0;
    virtual void banish() const = 0;

    virtual std::shared_ptr<const T> get_true_self() const noexcept = 0;
};

}
}
}
