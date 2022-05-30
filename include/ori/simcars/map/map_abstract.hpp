#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_interface.hpp>

#include <string>
#include <fstream>
#include <filesystem>
#include <memory>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id, class T_map>
class AMap : public IMap<T_id>, public std::enable_shared_from_this<AMap<T_id, T_map>>
{
protected:
    AMap() = default;

    virtual void save_virt(std::ofstream& output_filestream) const = 0;
    virtual void load_virt(std::ifstream& input_filestream) = 0;

public:
    ~AMap() override
    {
        static_assert(std::is_base_of<AMap, T_map>::value, "T_map is not derived from AMap");
    }

    void save(const std::string& output_file_path_str) const override
    {
        std::filesystem::path output_file_path(output_file_path_str);

        if (!std::filesystem::is_regular_file(output_file_path))
        {
            throw std::invalid_argument("Output file path '" + output_file_path_str + "' does not indicate a valid file");
        }

        std::ofstream output_filestream(output_file_path, std::ios_base::binary);

        this->save_virt(output_filestream);
    }

    virtual std::shared_ptr<T_map> copy() const = 0;

    static std::shared_ptr<const IMap<T_id>> load(const std::string& input_file_path_str)
    {
        std::filesystem::path input_file_path(input_file_path_str);

        if (!std::filesystem::is_regular_file(input_file_path))
        {
            throw std::invalid_argument("Input file path '" + input_file_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream(input_file_path, std::ios_base::binary);

        std::shared_ptr<AMap<T_id, T_map>> map(new T_map());

        map->load_virt(input_filestream);

        return map;
    }
};

}
}
}
