#include "rozum_pulse_75_hardware/rozum_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <curl/curl.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

constexpr double PI = 3.14159265358979323846;

namespace rozum_pulse_75_hardware
{

static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

hardware_interface::CallbackReturn RozumSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("base_url") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"), "Missing 'base_url' in URDF <ros2_control> params.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  base_url_ = info_.hardware_parameters.at("base_url");
  joint_positions_.assign(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RozumSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string url = base_url_ + "/status/motors";

  CURL* curl = curl_easy_init();
  if (!curl)
    return hardware_interface::return_type::ERROR;

  std::string readBuffer;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);

  CURLcode res = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"), "CURL request failed: %s", curl_easy_strerror(res));
    return hardware_interface::return_type::ERROR;
  }

  rapidjson::Document doc;
  doc.Parse(readBuffer.c_str());
  if (!doc.IsArray())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"), "Invalid JSON from %s", url.c_str());
    return hardware_interface::return_type::ERROR;
  }

  
  for (size_t i = 0; i < joint_positions_.size() && i < doc.Size(); ++i)
  {
    if (doc[i].HasMember("angle") && doc[i]["angle"].IsNumber()) {
        double deg = doc[i]["angle"].GetDouble();
        joint_positions_[i] = deg * PI / 180.0;

    }
      
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RozumSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}  // namespace rozum_pulse_75_hardware

PLUGINLIB_EXPORT_CLASS(rozum_pulse_75_hardware::RozumSystemHardware, hardware_interface::SystemInterface)