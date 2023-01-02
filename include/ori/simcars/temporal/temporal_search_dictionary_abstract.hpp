#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/temporal/typedefs.hpp>

#include <stdexcept>
#include <deque>
#include <unordered_map>
#include <set>

namespace ori
{
namespace simcars
{
namespace temporal
{

class TimeHasher
{
public:
    std::size_t operator()(Time const &key) const
    {
        return key.time_since_epoch().count();
    }
};

template <typename V>
class AbstractTemporalSearchDictionary : public virtual structures::stl::STLDictionary<Time, V, TimeHasher>
{
    size_t const max_cache_size;
    Duration const time_diff_threshold;

    std::set<Time> timestamp_ordered_set;

    mutable std::unordered_map<Time, Time, TimeHasher> closest_timestamp_cache_dict;
    mutable std::deque<Time> timestamp_cache_queue;

    void enforce_max_cache_size() const
    {
        while (timestamp_cache_queue.size() > max_cache_size)
        {
            Time timestamp = timestamp_cache_queue.front();
            timestamp_cache_queue.pop_front();
            closest_timestamp_cache_dict.erase(timestamp);
        }
    }

    virtual Time search(Time const &timestamp) const = 0;
    virtual bool search(Time const &timestamp, Time &closest_timestamp) const = 0;

public:
    AbstractTemporalSearchDictionary(size_t max_cache_size)
        : AbstractTemporalSearchDictionary<V>(Duration::max() / 2, max_cache_size) {}
    AbstractTemporalSearchDictionary(Duration time_diff_threshold, size_t max_cache_size)
        : time_diff_threshold(time_diff_threshold), max_cache_size(max_cache_size) {}
    AbstractTemporalSearchDictionary(AbstractTemporalSearchDictionary<V> const &temporal_dictionary)
        : structures::stl::STLDictionary<Time, V, TimeHasher>(temporal_dictionary),
          timestamp_ordered_set(temporal_dictionary.timestamp_ordered_set),
          time_diff_threshold(temporal_dictionary.time_diff_threshold),
          max_cache_size(temporal_dictionary.max_cache_size) {}

    bool contains(Time const &timestamp) const override
    {
        if (structures::stl::STLDictionary<Time, V, TimeHasher>::contains(timestamp))
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
            closest_timestamp_cache_dict[timestamp] = closest_timestamp;
            timestamp_cache_queue.push_back(timestamp);
            enforce_max_cache_size();
        }

        return found_match;
    }

    V const& operator [](Time const &timestamp) const override
    {
        if (structures::stl::STLDictionary<Time, V, TimeHasher>::contains(timestamp))
        {
            return structures::stl::STLDictionary<Time, V, TimeHasher>::operator [](timestamp);
        }

        Time closest_timestamp;

        if (closest_timestamp_cache_dict.contains(timestamp))
        {
            closest_timestamp = closest_timestamp_cache_dict[timestamp];
        }
        else
        {
            closest_timestamp = search(timestamp);

            closest_timestamp_cache_dict[timestamp] = closest_timestamp;
            timestamp_cache_queue.push_back(timestamp);
            enforce_max_cache_size();
        }

        return structures::stl::STLDictionary<Time, V, TimeHasher>::operator [](closest_timestamp);
    }
    structures::IArray<Time> const* get_keys() const override
    {
        return structures::stl::STLDictionary<Time, V, TimeHasher>::get_keys();
    }
    void get_keys(structures::IStackArray<Time> *keys) const override
    {
        size_t i = 0;
        for (Time const &time : timestamp_ordered_set)
        {
            if (i >= keys->count())
            {
                keys->push_back(time);
                ++i;
            }
            else
            {
                (*keys)[i] = time;
                ++i;
            }
        }
    }
    structures::IArray<V> const* get_values() const override
    {
        return structures::stl::STLDictionary<Time, V, TimeHasher>::get_values();
    }
    void get_values(structures::IStackArray<V> *values) const override
    {
        structures::IArray<Time> const *timestamps = structures::stl::STLDictionary<Time, V, TimeHasher>::get_keys();

        for (size_t i = 0; i < timestamps->count(); ++i)
        {
            Time timestamp = (*timestamps)[i];
            if (i >= values->count())
            {
                values->push_back(structures::stl::STLDictionary<Time, V, TimeHasher>::operator [](timestamp));
            }
            else
            {
                (*values)[i] = structures::stl::STLDictionary<Time, V, TimeHasher>::operator [](timestamp);
            }
        }
    }

    bool contains(Time const &timestamp, bool exact) const
    {
        if (exact)
        {
            return structures::stl::STLDictionary<Time, V, TimeHasher>::contains(timestamp);
        }
        else
        {
            return this->contains(timestamp);
        }
    }

    Time get_earliest_timestamp() const
    {
        if (this->data.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return *(timestamp_ordered_set.begin());
    }
    Time get_latest_timestamp() const
    {
        if (this->data.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return *(timestamp_ordered_set.rbegin());
    }
    Duration get_time_diff_threshold() const
    {
        return time_diff_threshold;
    }
    size_t get_max_cache_size() const
    {
        return max_cache_size;
    }

    void update(Time const &key, V const &val) override
    {
        structures::stl::STLDictionary<Time, V, TimeHasher>::update(key, val);
        timestamp_ordered_set.insert(key);
    }
    void erase(Time const &key) override
    {
        structures::stl::STLDictionary<Time, V, TimeHasher>::erase(key);
        timestamp_ordered_set.erase(key);
    }
};

}
}
}
