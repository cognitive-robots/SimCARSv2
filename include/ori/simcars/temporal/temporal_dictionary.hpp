#pragma once

#include <ori/simcars/structures/stl/stl_ordered_dictionary.hpp>
#include <ori/simcars/temporal/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename V>
class TemporalDictionary : public virtual structures::stl::STLOrderedDictionary<Time, V>
{
public:
    using structures::stl::STLOrderedDictionary<Time, V>::STLOrderedDictionary;

    bool contains(Time const &key) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(this->data_mutex);

        return key >= get_earliest_time();
    }

    V const& operator [](Time const &key) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(this->data_mutex);

        return (--this->data.upper_bound(key))->second;
    }

    Time get_most_recent_time(Time const &key) const
    {
        std::lock_guard<std::recursive_mutex> data_guard(this->data_mutex);

        return (--this->data.upper_bound(key))->first;
    }
    Time get_earliest_time() const
    {
        std::lock_guard<std::recursive_mutex> data_guard(this->data_mutex);

        return this->data.begin()->first;
    }
    Time get_latest_time() const
    {
        std::lock_guard<std::recursive_mutex> data_guard(this->data_mutex);

        return (--this->data.end())->first;
    }
};

}
}
}
