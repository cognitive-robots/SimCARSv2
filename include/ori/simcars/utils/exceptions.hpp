#pragma once

#include <exception>

namespace ori
{
namespace simcars
{
namespace utils
{

class NotImplementedException : public std::exception
{
public:
    using std::exception::exception;

    char const* what() const noexcept override
    {
        return "This functionality has not been implemented yet";
    }
};

}
}
}
