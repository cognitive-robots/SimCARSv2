#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/file_based_map_interface.hpp>

#include <string>
#include <fstream>
#include <filesystem>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id, class T_map>
class AFileBasedMap : public virtual IFileBasedMap<T_id>
{
protected:
    AFileBasedMap() = default;

    virtual void save_virt(std::ofstream &output_filestream) const = 0;
    virtual void load_virt(std::ifstream &input_filestream) = 0;

public:
    ~AFileBasedMap() override
    {
        static_assert(std::is_base_of<AFileBasedMap, T_map>::value, "T_map is not derived from AFileBasedMap");
    }

    void save(std::string const &output_file_path_str) const override
    {
        std::filesystem::path output_file_path(output_file_path_str);

        if (!std::filesystem::is_directory(output_file_path.parent_path()))
        {
            throw std::invalid_argument("Output file path directory '" + output_file_path.parent_path().string() + "' does not indicate a valid directory");
        }

        std::ofstream output_filestream(output_file_path, std::ios_base::binary);

        this->save_virt(output_filestream);
    }

    virtual T_map* shallow_copy() const = 0;

    static T_map const* load(std::string const &input_file_path_str)
    {
        std::filesystem::path input_file_path(input_file_path_str);

        if (!std::filesystem::is_regular_file(input_file_path))
        {
            throw std::invalid_argument("Input file path '" + input_file_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream(input_file_path, std::ios_base::binary);

        AFileBasedMap<T_id, T_map> *map = new T_map();

        map->load_virt(input_filestream);

        return dynamic_cast<T_map const*>(map);
    }
};

}
}
}
