#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_deque_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/temporal/typedefs.hpp>

#include <deque>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename V>
class TemporalRoundingDictionary : public virtual structures::IDictionary<Time, V>
{
    Time time_window_start;
    Duration time_window_step;
    V default_value;

    std::deque<V> value_deque;

    mutable structures::IStackArray<Time> *keys_cache;
    mutable structures::IStackArray<V> *values_cache;

public:
    TemporalRoundingDictionary(Duration time_window_step, V const &default_value)
        : time_window_step(time_window_step), default_value(default_value),
          keys_cache(nullptr), values_cache(nullptr) {}

    ~TemporalRoundingDictionary() override
    {
        delete keys_cache;
        delete values_cache;
    }

    size_t count() const override
    {
        return value_deque.size();
    }
    bool contains(Time const &key) const override
    {
        size_t index = (key - time_window_start).count() / time_window_step.count();
        if (key >= time_window_start && index >= 0 && index < value_deque.size())
        {
            return value_deque[index] != default_value;
        }

        return false;
    }

    V const& operator [](Time const &key) const override
    {
        if (key < time_window_start)
        {
            return default_value;
        }
        else
        {
            return value_deque[(key - time_window_start).count() / time_window_step.count()];
        }
    }
    bool contains_value(V const &val) const override
    {
        for (V const &value : value_deque)
        {
            if (value == val)
            {
                return true;
            }
        }

        return false;
    }
    structures::IArray<Time> const* get_keys() const override
    {
        if (keys_cache)
        {
            return keys_cache;
        }
        else
        {
            keys_cache = new structures::stl::STLStackArray<Time>;
            get_keys(keys_cache);
            return keys_cache;
        }
    }
    void get_keys(structures::IStackArray<Time> *keys) const override
    {
        structures::stl::STLSet<V> previous_values;
        size_t i, j;
        for (i = 0, j = 0; i < value_deque.size(); ++i, ++j)
        {
            if (previous_values.contains(value_deque[i]))
            {
                --j;
                continue;
            }

            if (j >= keys->count())
            {
                keys->push_back(time_window_start + i * time_window_step);
            }
            else
            {
                (*keys)[j] = time_window_start + i * time_window_step;
            }

            previous_values.insert(value_deque[i]);
        }
    }
    structures::IArray<V> const* get_values() const override
    {
        if (values_cache)
        {
            return values_cache;
        }
        else
        {
            values_cache = new structures::stl::STLStackArray<V>;
            get_values(values_cache);
            return values_cache;
        }
    }
    void get_values(structures::IStackArray<V> *values) const override
    {
        structures::stl::STLSet<V> previous_values;
        size_t i = 0;
        for (V const &value : value_deque)
        {
            if (previous_values.contains(value))
            {
                continue;
            }

            if (i >= values->count())
            {
                values->push_back(value);
                ++i;
            }
            else
            {
                (*values)[i] = value;
                ++i;
            }

            previous_values.insert(value);
        }
    }

    bool contains(Time const &key, bool exact) const
    {
        size_t index = (key - time_window_start).count() / time_window_step.count();

        if (key >= time_window_start &&
                (!exact || time_window_start + time_window_step * index == key) &&
                index >= 0 && index < value_deque.size())
        {
            return value_deque[index] != default_value;
        }

        return false;
    }

    Time get_earliest_timestamp() const
    {
        if (value_deque.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return time_window_start;
    }
    Time get_latest_timestamp() const
    {
        if (value_deque.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return time_window_start + time_window_step * (value_deque.size() - 1);
    }
    Duration get_time_window_step() const
    {
        return time_window_step;
    }
    V get_default_value() const
    {
        return default_value;
    }

    void update(Time const &key, V const &val) override
    {
        if (value_deque.size() == 0)
        {
            time_window_start = key;
            value_deque.push_back(val);
        }
        else
        {
            while (key < time_window_start)
            {
                value_deque.push_front(default_value);
                time_window_start -= time_window_step;
            }

            while (key >= time_window_start + time_window_step * value_deque.size())
            {
                value_deque.push_back(default_value);
            }

            size_t index = (key - time_window_start).count() / time_window_step.count();
            value_deque[index] = val;
        }

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
    void erase(Time const &key) override
    {
        size_t index = (key - time_window_start).count() / time_window_step.count();
        if (key >= time_window_start && index < 1)
        {
            value_deque.pop_front();
            time_window_start += time_window_step;
        }
        else if (index == value_deque.size() - 1)
        {
            value_deque.pop_back();
        }
        else if (index > 0 && index < value_deque.size() - 1)
        {
            value_deque[index] = default_value;
        }

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
    void propogate_values_forward()
    {
        V current_value = default_value;
        structures::stl::STLSet<V> previous_values;
        for (size_t i = 0; i < value_deque.size(); ++i)
        {
            if (value_deque[i] != current_value)
            {
                if (previous_values.contains(value_deque[i]))
                {
                    value_deque[i] = current_value;
                }
                else
                {
                    previous_values.insert(current_value);
                    current_value = value_deque[i];
                }
            }
        }

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
    void propogate_values_forward(Time const &time_window_end)
    {
        if (value_deque.size() == 0)
        {
            return;
        }

        V current_value = default_value;
        structures::stl::STLSet<V> previous_values;
        Time current_time;
        size_t i;
        for (i = 0, current_time = time_window_start;
             current_time <= time_window_end;
             current_time += time_window_step, ++i)
        {
            if (i >= value_deque.size())
            {
                value_deque.push_back(current_value);
            }
            else
            {
                if (value_deque[i] != current_value)
                {
                    if (previous_values.contains(value_deque[i]))
                    {
                        value_deque[i] = current_value;
                    }
                    else
                    {
                        previous_values.insert(current_value);
                        current_value = value_deque[i];
                    }
                }
            }
        }

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
};

}
}
}
