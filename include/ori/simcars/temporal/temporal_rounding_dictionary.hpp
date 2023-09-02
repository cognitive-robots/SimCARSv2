#pragma once

#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/temporal/typedefs.hpp>

#include <deque>
#include <mutex>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename V>
class TemporalRoundingDictionary : public virtual structures::IDictionary<Time, V>
{
    Duration const time_window_step_size;
    V const default_value;

    Time time_window_start;

    std::deque<V> value_deque;

    mutable std::recursive_mutex value_deque_mutex;

    mutable structures::IStackArray<Time> *keys_cache;
    mutable structures::IStackArray<V> *values_cache;

public:
    TemporalRoundingDictionary(Duration time_window_step_size, V const &default_value)
        : time_window_step_size(time_window_step_size), default_value(default_value),
          keys_cache(nullptr), values_cache(nullptr) {}

    ~TemporalRoundingDictionary() override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        delete keys_cache;
        delete values_cache;
    }

    size_t count() const override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        return value_deque.size();
    }
    bool contains(Time const &key) const override
    {
        return contains(key, false);
    }

    V const& operator [](Time const &key) const override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        if (key < time_window_start)
        {
            return default_value;
        }
        else
        {
            return value_deque[size_t((key - time_window_start).count() /
                                      time_window_step_size.count())];
        }
    }
    bool contains_value(V const &val) const override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

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
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

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
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        V current_value = default_value;
        size_t i, j;
        for (i = 0, j = 0; i < value_deque.size(); ++i, ++j)
        {
            if (value_deque[i] == current_value)
            {
                --j;
                continue;
            }
            else
            {
                current_value = value_deque[i];

                if (current_value == default_value)
                {
                    --j;
                    continue;
                }
            }

            if (j >= keys->count())
            {
                keys->push_back(time_window_start + i * time_window_step_size);
            }
            else
            {
                (*keys)[j] = time_window_start + i * time_window_step_size;
            }
        }
    }
    structures::IArray<V> const* get_values() const override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

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
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        values->clear();

        for (V const &value : value_deque)
        {
            if (values->contains(value))
            {
                continue;
            }

            values->push_back(value);
        }
    }

    bool contains(Time const &key, bool exact) const
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        size_t index = size_t((key - time_window_start).count() / time_window_step_size.count());

        if (key >= time_window_start &&
                (!exact || time_window_start + time_window_step_size * index == key) &&
                index >= 0 && index < value_deque.size())
        {
            return value_deque[index] != default_value;
        }

        return false;
    }

    Time get_earliest_timestamp() const
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        if (value_deque.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return time_window_start;
    }
    Time get_latest_timestamp() const
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        if (value_deque.size() == 0)
        {
            throw std::out_of_range("Temporal dictionary is empty");
        }

        return time_window_start + time_window_step_size * (value_deque.size() - 1);
    }
    Duration get_time_window_step_size() const
    {
        return time_window_step_size;
    }
    V get_default_value() const
    {
        return default_value;
    }

    void update(Time const &key, V const &val) override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

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
                time_window_start -= time_window_step_size;
            }

            while (key >= time_window_start + time_window_step_size * value_deque.size())
            {
                value_deque.push_back(default_value);
            }

            size_t index = size_t((key - time_window_start).count() /
                                  time_window_step_size.count());
            value_deque[index] = val;
        }

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
    void erase(Time const &key) override
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        size_t index = size_t((key - time_window_start).count() / time_window_step_size.count());
        if (key >= time_window_start && index < 1)
        {
            V value_to_erase = value_deque[0];
            while (value_deque[0] == value_to_erase)
            {
                value_deque.pop_front();
                time_window_start += time_window_step_size;
            }

            delete keys_cache;
            keys_cache = nullptr;

            delete values_cache;
            values_cache = nullptr;
        }
        else if (key >= time_window_start && index == value_deque.size() - 1)
        {
            V value_to_erase = value_deque[value_deque.size() - 1];
            while (value_deque[value_deque.size() - 1] == value_to_erase)
            {
                value_deque.pop_back();
            }

            delete keys_cache;
            keys_cache = nullptr;

            delete values_cache;
            values_cache = nullptr;
        }
        else if (index > 0 && index < value_deque.size() - 1)
        {
            V value_to_erase = value_deque[index];
            for (size_t i = index; i < value_deque.size(); ++i)
            {
                if (value_deque[i] == value_to_erase)
                {
                    if (i == value_deque.size() - 1)
                    {
                        value_deque.resize(index);
                        break;
                    }
                    else
                    {
                        value_deque[i] = default_value;
                    }
                }
                else
                {
                    break;
                }
            }

            delete keys_cache;
            keys_cache = nullptr;

            delete values_cache;
            values_cache = nullptr;
        }
    }
    void propogate_values_forward()
    {
        propogate_values_forward(get_latest_timestamp());
    }
    void propogate_values_forward(Time const &time_window_end)
    {
        std::lock_guard<std::recursive_mutex> value_deque_guard(value_deque_mutex);

        V current_value = default_value;
        Time current_time;
        size_t i;
        for (i = 0, current_time = time_window_start;
             current_time <= time_window_end;
             current_time += time_window_step_size, ++i)
        {
            if (i >= value_deque.size())
            {
                value_deque.push_back(current_value);
            }
            else
            {
                if (value_deque[i] == default_value)
                {
                    value_deque[i] = current_value;
                }
                else
                {
                    current_value = value_deque[i];
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
