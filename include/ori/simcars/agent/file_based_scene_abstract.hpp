#pragma once

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/set_interface.hpp>
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

    virtual void save_virt(std::ofstream &output_filestream) const = 0;
    virtual void load_virt(std::ifstream &input_filestream, structures::ISet<std::string>* agent_names) = 0;

public:
    ~AFileBasedScene() override
    {
        static_assert(std::is_base_of<AFileBasedScene, T_scene>::value, "T_map is not derived from AFileBasedScene");
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

    static T_scene const* load(std::string const &input_file_path_str, structures::ISet<std::string>* agent_names = nullptr)
    {
        std::filesystem::path input_file_path(input_file_path_str);

        if (!std::filesystem::is_regular_file(input_file_path))
        {
            throw std::invalid_argument("Input file path '" + input_file_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream(input_file_path, std::ios_base::binary);

        AFileBasedScene<T_scene> *scene = new T_scene;

        scene->load_virt(input_filestream, agent_names);

        return dynamic_cast<T_scene const*>(scene);
    }
};

}
}
}
