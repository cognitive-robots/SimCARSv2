#pragma once

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/scene_interface.hpp>

#include <fstream>
#include <filesystem>

namespace ori
{
namespace simcars
{
namespace agent
{

template <class T_scene>
class AScene : public IScene, public std::enable_shared_from_this<AScene<T_scene>>
{
protected:
    AScene() = default;

    virtual void save_virt(std::ofstream& output_filestream) const = 0;
    virtual void load_virt(std::ifstream& input_filestream) = 0;

public:
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

    static std::shared_ptr<const IScene> load(const std::string& input_file_path_str)
    {
        std::filesystem::path input_file_path(input_file_path_str);

        if (!std::filesystem::is_regular_file(input_file_path))
        {
            throw std::invalid_argument("Input file path '" + input_file_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream(input_file_path, std::ios_base::binary);

        std::shared_ptr<AScene<T_scene>> scene(new T_scene());

        scene->load_virt(input_filestream);

        return scene;
    }
};

}
}
}
