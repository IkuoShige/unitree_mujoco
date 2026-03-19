#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <array>
#include <vector>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    std::string track_body; // body name to track camera, empty = no tracking
    bool stage2_random_ball_on_enter = false;
    std::array<float, 2> stage2_spawn_angle_range = {-0.30f, 0.30f};
    std::array<float, 2> stage2_spawn_radius_range = {-0.05f, 0.35f};
    float stage2_min_ball_distance = 0.25f;
    bool stage2_enforce_forward_spawn = true;
    float stage2_min_forward_spawn_x = 0.05f;
    float stage2_ball_height = 0.11f;
    bool stage2_rolling_ball_on_enter = false;
    std::array<float, 2> stage2_rolling_speed_range = {0.1f, 0.3f};

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if(cfg["track_body"]) {
                track_body = cfg["track_body"].as<std::string>();
            }
            if (cfg["stage2_random_ball_on_enter"]) {
                stage2_random_ball_on_enter = cfg["stage2_random_ball_on_enter"].as<bool>();
            }
            if (cfg["stage2_spawn_angle_range"]) {
                auto v = cfg["stage2_spawn_angle_range"].as<std::vector<float>>();
                if (v.size() == 2) {
                    stage2_spawn_angle_range = {v[0], v[1]};
                }
            }
            if (cfg["stage2_spawn_radius_range"]) {
                auto v = cfg["stage2_spawn_radius_range"].as<std::vector<float>>();
                if (v.size() == 2) {
                    stage2_spawn_radius_range = {v[0], v[1]};
                }
            }
            if (cfg["stage2_min_ball_distance"]) {
                stage2_min_ball_distance = cfg["stage2_min_ball_distance"].as<float>();
            }
            if (cfg["stage2_enforce_forward_spawn"]) {
                stage2_enforce_forward_spawn = cfg["stage2_enforce_forward_spawn"].as<bool>();
            }
            if (cfg["stage2_min_forward_spawn_x"]) {
                stage2_min_forward_spawn_x = cfg["stage2_min_forward_spawn_x"].as<float>();
            }
            if (cfg["stage2_ball_height"]) {
                stage2_ball_height = cfg["stage2_ball_height"].as<float>();
            }
            if (cfg["stage2_rolling_ball_on_enter"]) {
                stage2_rolling_ball_on_enter = cfg["stage2_rolling_ball_on_enter"].as<bool>();
            }
            if (cfg["stage2_rolling_speed_range"]) {
                auto v = cfg["stage2_rolling_speed_range"].as<std::vector<float>>();
                if (v.size() == 2) {
                    stage2_rolling_speed_range = {v[0], v[1]};
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
