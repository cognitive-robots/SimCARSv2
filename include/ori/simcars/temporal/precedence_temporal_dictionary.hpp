#pragma once

#include <ori/simcars/structures/stl/stl_ordered_dictionary.hpp>
#include <ori/simcars/structures/stl/stl_queue_array.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/temporal/temporal_dictionary_abstract.hpp>

#include <stdexcept>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename V>
class PrecedenceTemporalDictionary : public virtual AbstractTemporalDictionary<V>
{
    Time search(const Time& timestamp) const override
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

        if (timestamp < timestamps[search_window_start])
        {
            throw std::out_of_range("Specified timestamp is outside the period covered by the temporal dictionary");
        }

        if (timestamp >= timestamps[search_window_end - 1])
        {
            return timestamps[search_window_end - 1];
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

        return timestamps[search_window_start];
    }
    bool search(const Time& timestamp, Time& closest_timestamp) const override
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

        if (timestamp < timestamps[search_window_start])
        {
            return false;
        }

        if (timestamp >= timestamps[search_window_end - 1])
        {
            closest_timestamp = timestamps[search_window_end - 1];
            return true;
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

        closest_timestamp = timestamps[search_window_start];
        return true;
    }

public:
    PrecedenceTemporalDictionary(size_t max_cache_size)
        : AbstractTemporalDictionary<V>(max_cache_size) {}
    PrecedenceTemporalDictionary(const PrecedenceTemporalDictionary<V>& temporal_dictionary)
        : AbstractTemporalDictionary<V>(temporal_dictionary) {}

};

}
}
}
