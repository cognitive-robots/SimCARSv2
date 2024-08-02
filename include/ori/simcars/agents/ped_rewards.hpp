#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedRewards
{
    friend std::ostream& operator<<(std::ostream& output_stream, const PedRewards& rewards)
    {
        return output_stream;
    }
};

}
}
}
