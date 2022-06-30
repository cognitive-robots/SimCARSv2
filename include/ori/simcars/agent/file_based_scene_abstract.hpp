#pragma once

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/file_based_scene_interface.hpp>
#include <ori/simcars/agent/scene_abstract.hpp>

#include <fstream>
#include <filesystem>

namespace ori
{
namespace simcars
{
namespace agent
{

template <class T_scene>
class AFileBasedScene : public virtual AScene, public virtual IFileBasedScene
{
protected:
    AFileBasedScene() = default;

    virtual void save_virt(std::ofstream& output_filestream) const = 0;
    virtual void load_virt(std::ifstream& input_filestream) = 0;

public:
    ~AFileBasedScene() override
    {
        static_assert(std::is_base_of<AFileBasedScene, T_scene>::value, "T_map is not derived from AFileBasedScene");
    }

    void save(const std::string& output_file_path_str) const override
    {
        std::filesystem::path output_file_path(output_file_path_str);

        if (!std::filesystem::is_directory(output_file_path.parent_path()))
        {
            throw std::invalid_argument("Output file path directory '" + output_file_path.parent_path().string() + "' does not indicate a valid directory");
        }

        std::ofstream output_filestream(output_file_path, std::ios_base::binary);

        this->save_virt(output_filestream);
    }

    static std::shared_ptr<const T_scene> load(const std::string& input_file_path_str)
    {
        std::filesystem::path input_file_path(input_file_path_str);

        if (!std::filesystem::is_regular_file(input_file_path))
        {
            throw std::invalid_argument("Input file path '" + input_file_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream(input_file_path, std::ios_base::binary);

        std::shared_ptr<AFileBasedScene<T_scene>> scene(new T_scene());

        scene->load_virt(input_filestream);

        return std::dynamic_pointer_cast<const T_scene>(scene);
    }
};

}
}
}
