#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/temporal/temporal_search_dictionary_abstract.hpp>

#include <stdexcept>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename V>
class ProximalTemporalDictionary : public virtual AbstractTemporalSearchDictionary<V>
{
    Time search(Time const &timestamp) const override
    {
        structures::IArray<Time> const &timestamps = *(this->get_keys());

        size_t search_window_start = 0;
        size_t search_window_end = timestamps.count();
        size_t idx;
        Time current_timestamp;

        if (search_window_end - search_window_start == 0)
        {
           throw std::out_of_range("Temporal dictionary is empty");
        }

        if (timestamp < timestamps[search_window_start] - this->get_time_diff_threshold()
                || timestamp > timestamps[search_window_end - 1] + this->get_time_diff_threshold())
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

        if (min_diff > this->get_time_diff_threshold())
        {
            throw std::out_of_range("Specified timestamp is too far from the closest timestamp to be considered a match");
        }

        return closest_timestamp;
    }
    bool search(Time const &timestamp, Time &closest_timestamp) const override
    {
        structures::IArray<Time> const &timestamps = *(this->get_keys());

        size_t search_window_start = 0;
        size_t search_window_end = timestamps.count();
        size_t idx;
        Time current_timestamp;

        if (search_window_end - search_window_start == 0)
        {
            return false;
        }

        if (timestamp < timestamps[search_window_start] - this->get_time_diff_threshold()
                || timestamp > timestamps[search_window_end - 1] + this->get_time_diff_threshold())
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

        if (min_diff > this->get_time_diff_threshold())
        {
            return false;
        }

        return true;
    }

public:
    ProximalTemporalDictionary(size_t max_cache_size)
        : ProximalTemporalDictionary<V>(Duration::max() / 2, max_cache_size) {}
    ProximalTemporalDictionary(Duration time_diff_threshold, size_t max_cache_size)
        : AbstractTemporalSearchDictionary<V>(time_diff_threshold, max_cache_size) {}
    ProximalTemporalDictionary(ProximalTemporalDictionary<V> const &temporal_dictionary)
        : AbstractTemporalSearchDictionary<V>(temporal_dictionary) {}

};

}
}
}
