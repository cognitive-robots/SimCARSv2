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
class TemporalDictionary : public virtual structures::stl::STLOrderedDictionary<Time, V>
{
    const size_t max_cache_size;
    mutable structures::stl::STLOrderedDictionary<Time, Time> closest_timestamp_cache_dict;
    mutable structures::stl::STLQueueArray<Time> timestamp_cache_queue;

    const Duration time_diff_threshold;

    void enforce_max_cache_size() const
    {
        while (timestamp_cache_queue.count() > max_cache_size)
        {
            Time timestamp = timestamp_cache_queue.pop_front();
            closest_timestamp_cache_dict.erase(timestamp);
        }
    }

    Time search(const Time& timestamp) const
    {
        const structures::IArray<Time>& timestamps = *(this->get_keys());

        size_t search_window_start = 0;
        size_t search_window_end = timestamps.count();
        size_t idx;
        Time current_timestamp;

        if (search_window_end - search_window_start == 0)
        {
           throw std::out_of_range("Temporal dictionary is empty");
        }

        if (timestamp < timestamps[search_window_start] - time_diff_threshold
                || timestamp > timestamps[search_window_end - 1] + time_diff_threshold)
        {
            throw std::out_of_range("Specified timestamp is outside the period covered by the temporal dictionary");
        }

        while (search_window_end - search_window_start > 1)
        {
            idx = search_window_start + (search_window_end - search_window_start) / 2;
            current_timestamp = timestamps[idx];

            if (current_timestamp <= timestamp)
            {
                search_window_start = idx;
            }
            else
            {
                search_window_end = idx;
            }
        }

        if (search_window_end == timestamps.count())
        {
            return timestamps[search_window_start];
        }

        Duration search_window_start_diff = timestamp - timestamps[search_window_start];
        Duration search_window_end_diff = timestamps[search_window_end] - timestamp;
        Duration min_diff;
        Time closest_timestamp;

        if (search_window_start_diff <= search_window_end_diff)
        {
            min_diff = search_window_start_diff;
            closest_timestamp = timestamps[search_window_start];
        }
        else
        {
            min_diff = search_window_end_diff;
            closest_timestamp = timestamps[search_window_end];
        }

        if (min_diff > time_diff_threshold)
        {
            throw std::out_of_range("Specified timestamp is too far from the closest timestamp to be considered a match");
        }

        return closest_timestamp;
    }
    bool search(const Time& timestamp, Time& closest_timestamp) const
    {
        const structures::IArray<Time>& timestamps = *(this->get_keys());

        size_t search_window_start = 0;
        size_t search_window_end = timestamps.count();
        size_t idx;
        Time current_timestamp;

        if (search_window_end - search_window_start == 0)
        {
            return false;
        }

        if (timestamp < timestamps[search_window_start] - time_diff_threshold
                || timestamp > timestamps[search_window_end - 1] + time_diff_threshold)
        {
            return false;
        }

        while (search_window_end - search_window_start > 1)
        {
            idx = search_window_start + (search_window_end - search_window_start) / 2;
            current_timestamp = timestamps[idx];

            if (current_timestamp <= timestamp)
            {
                search_window_start = idx;
            }
            else
            {
                search_window_end = idx;
            }
        }

        if (search_window_end == timestamps.count())
        {
            closest_timestamp = timestamps[search_window_start];
            return true;
        }

        Duration search_window_start_diff = timestamp - timestamps[search_window_start];
        Duration search_window_end_diff = timestamps[search_window_end] - timestamp;
        Duration min_diff;

        if (search_window_start_diff <= search_window_end_diff)
        {
            min_diff = search_window_start_diff;
            closest_timestamp = timestamps[search_window_start];
        }
        else
        {
            min_diff = search_window_end_diff;
            closest_timestamp = timestamps[search_window_end];
        }

        if (min_diff > time_diff_threshold)
        {
            return false;
        }

        return true;
    }

public:
    TemporalDictionary(Duration time_diff_threshold, size_t max_cache_size)
        : structures::stl::STLOrderedDictionary<Time, V>(), max_cache_size(max_cache_size), time_diff_threshold(time_diff_threshold) {}
    TemporalDictionary(const TemporalDictionary<V>& temporal_dictionary)
        : structures::stl::STLOrderedDictionary<Time, V>(temporal_dictionary),
          max_cache_size(temporal_dictionary.max_cache_size), time_diff_threshold(temporal_dictionary.time_diff_threshold) {}

    bool contains(const Time& timestamp) const override
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

    const V& operator [](const Time& timestamp) const override
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
        const structures::IArray<Time>& timestamps = *(this->get_keys());

        if (timestamps.count() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return timestamps[0];
    }
    Time get_latest_timestamp() const
    {
        const structures::IArray<Time>& timestamps = *(this->get_keys());

        if (timestamps.count() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return timestamps[timestamps.count() - 1];
    }
};

}
}
}
