#pragma once

#include <ori/simcars/map/map_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class IFileBasedMap : public virtual IMap<T_id>
{
public:
    virtual void save(std::string const &output_file_path_str) const = 0;
};

}
}
}
