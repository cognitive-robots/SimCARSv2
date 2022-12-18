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

inline std::ostream& operator <<(std::ostream &output_stream, temporal::Duration const &duration)
{
    return output_stream << std::to_string(duration.count());
}

inline std::ostream& operator <<(std::ostream &output_stream, temporal::Time const &time)
{
    return output_stream << time.time_since_epoch();
}

}
}
