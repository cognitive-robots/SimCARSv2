#pragma once

#include <chrono>

namespace ori
{
namespace simcars
{
namespace temporal
{

typedef int64_t DurationRep;
typedef std::chrono::duration<DurationRep, std::milli> Duration;
typedef std::chrono::time_point<std::chrono::steady_clock, Duration> Time;

}
}
}
