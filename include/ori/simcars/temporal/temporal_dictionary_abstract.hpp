#pragma once

#include <ori/simcars/structures/stl/stl_ordered_dictionary.hpp>
#include <ori/simcars/structures/stl/stl_queue_array.hpp>
#include <ori/simcars/temporal/typedefs.hpp>

#include <stdexcept>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename V>
class AbstractTemporalDictionary : public virtual structures::stl::STLOrderedDictionary<Time, V>
{
    size_t const max_cache_size;
    Duration const time_diff_threshold;
    mutable structures::stl::STLOrderedDictionary<Time, Time> closest_timestamp_cache_dict;
    mutable structures::stl::STLQueueArray<Time> timestamp_cache_queue;

    void enforce_max_cache_size() const
    {
        while (timestamp_cache_queue.count() > max_cache_size)
        {
            Time timestamp = timestamp_cache_queue.pop_front();
            closest_timestamp_cache_dict.erase(timestamp);
        }
    }

    virtual Time search(Time const &timestamp) const = 0;
    virtual bool search(Time const &timestamp, Time &closest_timestamp) const = 0;

public:
    AbstractTemporalDictionary(size_t max_cache_size)
        : AbstractTemporalDictionary<V>(Duration::max() / 2, max_cache_size) {}
    AbstractTemporalDictionary(Duration time_diff_threshold, size_t max_cache_size)
        : time_diff_threshold(time_diff_threshold), max_cache_size(max_cache_size) {}
    AbstractTemporalDictionary(AbstractTemporalDictionary<V> const &temporal_dictionary)
        : structures::stl::STLOrderedDictionary<Time, V>(temporal_dictionary),
          time_diff_threshold(temporal_dictionary.time_diff_threshold),
          max_cache_size(temporal_dictionary.max_cache_size) {}

    bool contains(Time const &timestamp) const override
    {
        if (structures::stl::STLOrderedDictionary<Time, V>::contains(timestamp))
        {
            return true;
        }

        if (closest_timestamp_cache_dict.contains(timestamp))
        {
            return true;
        }

        Time closest_timestamp;
        bool found_match = search(timestamp, closest_timestamp);

        if (found_match)
        {
            closest_timestamp_cache_dict.update(timestamp, closest_timestamp);
            timestamp_cache_queue.push_back(timestamp);
            enforce_max_cache_size();
        }

        return found_match;
    }

    V const& operator [](Time const &timestamp) const override
    {
        if (structures::stl::STLOrderedDictionary<Time, V>::contains(timestamp))
        {
            return structures::stl::STLOrderedDictionary<Time, V>::operator [](timestamp);
        }

        Time closest_timestamp;

        if (closest_timestamp_cache_dict.contains(timestamp))
        {
            closest_timestamp = closest_timestamp_cache_dict[timestamp];
        }
        else
        {
            closest_timestamp = search(timestamp);

            closest_timestamp_cache_dict.update(timestamp, closest_timestamp);
            timestamp_cache_queue.push_back(timestamp);
            enforce_max_cache_size();
        }

        return this->data.at(closest_timestamp);
    }

    Time get_earliest_timestamp() const
    {
        if (this->data.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return this->data.begin()->first;
    }
    Time get_latest_timestamp() const
    {
        if (this->data.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return this->data.end()->first;
    }
    Duration get_time_diff_threshold() const
    {
        return time_diff_threshold;
    }
    size_t get_max_cache_size() const
    {
        return max_cache_size;
    }
};

}
}
}
