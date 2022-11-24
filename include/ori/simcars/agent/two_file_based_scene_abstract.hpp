#pragma once

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/set_interface.hpp>
#include <ori/simcars/agent/two_file_based_scene_interface.hpp>
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
class ATwoFileBasedScene : public virtual AScene, public virtual ITwoFileBasedScene
{
protected:
    ATwoFileBasedScene() = default;

    virtual void save_virt(std::ofstream &output_filestream_1, std::ofstream &output_filestream_2) const = 0;
    virtual void load_virt(std::ifstream &input_filestream_1, std::ifstream &input_filestream_2,
                           structures::ISet<std::string>* agent_names) = 0;

public:
    ~ATwoFileBasedScene() override
    {
        static_assert(std::is_base_of<ATwoFileBasedScene, T_scene>::value, "T_map is not derived from ATwoFileBasedScene");
    }

    void save(std::string const &output_file_1_path_str, std::string const &output_file_2_path_str) const override
    {
        std::filesystem::path output_file_1_path(output_file_1_path_str);

        if (!std::filesystem::is_directory(output_file_1_path.parent_path()))
        {
            throw std::invalid_argument("Output file 1 path directory '" + output_file_1_path.parent_path().string() + "' does not indicate a valid directory");
        }

        std::ofstream output_filestream_1(output_file_1_path, std::ios_base::binary);

        std::filesystem::path output_file_2_path(output_file_2_path_str);

        if (!std::filesystem::is_directory(output_file_2_path.parent_path()))
        {
            throw std::invalid_argument("Output file 2 path directory '" + output_file_2_path.parent_path().string() + "' does not indicate a valid directory");
        }

        std::ofstream output_filestream_2(output_file_2_path, std::ios_base::binary);

        this->save_virt(output_filestream_1, output_filestream_2);
    }

    static T_scene const* load(std::string const &input_file_1_path_str, std::string const &input_file_2_path_str,
                               structures::ISet<std::string>* agent_names = nullptr)
    {
        std::filesystem::path input_file_1_path(input_file_1_path_str);

        if (!std::filesystem::is_regular_file(input_file_1_path))
        {
            throw std::invalid_argument("Input file 1 path '" + input_file_1_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream_1(input_file_1_path, std::ios_base::binary);

        std::filesystem::path input_file_2_path(input_file_2_path_str);

        if (!std::filesystem::is_regular_file(input_file_2_path))
        {
            throw std::invalid_argument("Input file 2 path '" + input_file_2_path_str + "' does not indicate a valid file");
        }

        std::ifstream input_filestream_2(input_file_2_path, std::ios_base::binary);

        ATwoFileBasedScene<T_scene> *scene = new T_scene;

        scene->load_virt(input_filestream_1, input_filestream_2, agent_names);

        return dynamic_cast<T_scene const*>(scene);
    }
};

}
}
}
