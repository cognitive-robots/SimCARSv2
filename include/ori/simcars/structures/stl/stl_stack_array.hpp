#pragma once

#include <ori/simcars/structures/stack_array_interface.hpp>
#include <ori/simcars/structures/ra_mod_list_interface.hpp>

#include <vector>
#include <mutex>

namespace ori
{
namespace simcars
{
namespace structures
{
namespace stl
{

template <typename T>
class STLStackArray : public virtual IStackArray<T>, public virtual IRAModList<T>
{
protected:
    std::vector<T> data;

    mutable std::recursive_mutex data_mutex;

public:
    STLStackArray() {}
    STLStackArray(size_t size, T const &default_value = T()) : data(size, default_value) {}
    STLStackArray(std::initializer_list<T> init_list) : data(init_list) {}
    STLStackArray(STLStackArray<T> const &stl_stack_array) : data(stl_stack_array.data) {}
    STLStackArray(IArray<T> const *array) : data(array->count())
    {
        for (size_t i = 0; i < array->count(); ++i)
        {
            data[i] = (*array)[i];
        }
    }

    size_t count() const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.size();
    }
    bool contains(T const &val) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        for (T const &data_val : data)
        {
            if (data_val == val)
            {
                return true;
            }
        }
        return false;
    }

    T const& peek_back() const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.back();
    }

    T const& operator [](size_t idx) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.at(idx);
    }

    STLStackArray<T>& operator =(STLStackArray<T> const &array)
    {
        std::lock_guard<std::recursive_mutex> this_data_guard(this->data_mutex);
        std::lock_guard<std::recursive_mutex> array_data_guard(array.data_mutex);

        this->data.resize(array.data.size());

        for (size_t i = 0; i < data.size(); ++i)
        {
            this->data[i] = array.data[i];
        }

        return *this;
    }

    void push_back(T const &val) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.push_back(val);
    }
    void clear() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.clear();
    }
    void resize(size_t size) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.resize(size);
    }

    void erase_back() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.pop_back();
    }
    T pop_back() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        T const val = peek_back();
        data.pop_back();
        return val;
    }

    T& operator [](size_t idx) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.at(idx);
    }

    void push_at(size_t idx, T const &val) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.insert(std::next(data.begin(), idx), val);
    }
    void erase_at(size_t idx) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.erase(std::next(data.begin(), idx));
    }
};

}
}
}
}
