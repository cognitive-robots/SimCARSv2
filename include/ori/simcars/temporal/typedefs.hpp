#pragma once

#include <chrono>
#include <ostream>

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

inline std::ostream& operator <<(std::ostream& output_stream, const temporal::Duration& duration)
{
    return output_stream << std::to_string(duration.count());
}

}
}
