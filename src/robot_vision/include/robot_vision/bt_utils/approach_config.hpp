#pragma once

#include <stdexcept>
#include <string>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

struct ApproachConfig
{
    double coarse_tolerance_m  {0.8};
    double trigger_distance_m  {0.6};
    double trigger_lateral_m   {0.15};
    double max_linear_speed    {0.18};
    double max_angular_speed   {0.6};
    double kp_rot              {1.2};
    double kp_fwd              {0.45};
    double search_speed        {0.4};
    double coarse_timeout_s    {60.0};
    double fine_timeout_s      {30.0};
    double search_timeout_s    {25.0};
    double lost_timeout_s      {2.0};
    double tick_hz             {20.0};
    double heading_threshold   {0.15};

    template<typename T>
    T getYamlValue(const YAML::Node& node, const char* key, const std::filesystem::path& path, T default_value) {
        const auto child = node[key];
        if (!child) {
            RCLCPP_WARN(rclcpp::get_logger("ApproachConfig"),
                "Key '%s' not found in approach config '%s', using default: %g",
                key, path.string().c_str(), default_value);
            return default_value;
        }

        try {
            return child.as<T>();
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("ApproachConfig"),
                "Invalid value for key '%s' in approach config '%s': %s. Using default: %g",
                key, path.string().c_str(), e.what(), default_value);
            return default_value;
        }
    }

    void loadApproachConfig(const std::filesystem::path& path) {
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("Approach config file not found: " + path.string());
        }

        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node node = root["approach"] ? root["approach"] : root;
        if (!node || !node.IsMap()) {
            throw std::runtime_error("Invalid approach config format in: " + path.string());
        }

        coarse_tolerance_m = getYamlValue<double>(node, "coarse_tolerance_m", path, 0.8);
        trigger_distance_m = getYamlValue<double>(node, "trigger_distance_m", path, 0.6);
        trigger_lateral_m  = getYamlValue<double>(node, "trigger_lateral_m",  path, 0.15);
        max_linear_speed   = getYamlValue<double>(node, "max_linear_speed",   path, 0.18);
        max_angular_speed  = getYamlValue<double>(node, "max_angular_speed",  path, 0.6);
        kp_rot             = getYamlValue<double>(node, "kp_rot",             path, 1.2);
        kp_fwd             = getYamlValue<double>(node, "kp_fwd",             path, 0.45);
        search_speed       = getYamlValue<double>(node, "search_speed",       path, 0.4);
        coarse_timeout_s   = getYamlValue<double>(node, "coarse_timeout_s",   path, 60.0);
        fine_timeout_s     = getYamlValue<double>(node, "fine_timeout_s",     path, 30.0);
        search_timeout_s   = getYamlValue<double>(node, "search_timeout_s",   path, 25.0);
        lost_timeout_s     = getYamlValue<double>(node, "lost_timeout_s",     path, 2.0);
        tick_hz            = getYamlValue<double>(node, "tick_hz",            path, 20.0);
        heading_threshold  = getYamlValue<double>(node, "heading_threshold",  path, 0.15);

        if (tick_hz <= 0.0) {
            throw std::runtime_error("Invalid approach.tick_hz in " + path.string() + ": must be > 0");
        }

        RCLCPP_INFO(rclcpp::get_logger("ApproachConfig"), "Loaded approach config from '%s'", path.string().c_str());
    }
};
