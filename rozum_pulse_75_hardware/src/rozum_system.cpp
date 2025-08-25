#include "rozum_pulse_75_hardware/rozum_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <curl/curl.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <iostream>
#include <string>

constexpr double PI = 3.14159265358979323846;

namespace rozum_pulse_75_hardware
{

static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

// std::vector<hardware_interface::CommandInterface> RozumSystemHardware::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> out;
//   out.reserve(info_.joints.size());

//   for (size_t i = 0; i < info_.joints.size(); ++i)
//   {
//     const auto & name = info_.joints[i].name; // "joint1".."joint6"
//     out.emplace_back(name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]);
//   }
//   return out;
// }

hardware_interface::CallbackReturn RozumSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  if (info_.hardware_parameters.find("base_url") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "Missing 'base_url' in URDF <ros2_control> params.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  base_url_ = info_.hardware_parameters.at("base_url");
  joint_positions_.assign(info_.joints.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("RozumSystemHardware"),
              "Initialized with base_url='%s', joints=%zu",
              base_url_.c_str(), info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RozumSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  //const std::string url = base_url_ + "/status/motors";
  const std::string url = "http://10.10.10.20:8081/pose";

  CURL* curl = curl_easy_init();
  if (!curl) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "curl_easy_init() failed");
    return hardware_interface::return_type::ERROR;
  }

  std::string readBuffer;
  char errbuf[CURL_ERROR_SIZE] = {0};

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
 
  struct curl_slist* hdrs = nullptr;
  hdrs = curl_slist_append(hdrs, "Accept: application/json");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);

  CURLcode res = curl_easy_perform(curl);

  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  
  curl_slist_free_all(hdrs);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "CURL request failed (%d): %s ; url=%s",
                 static_cast<int>(res),
                 errbuf[0] ? errbuf : curl_easy_strerror(res),
                 url.c_str());
    return hardware_interface::return_type::ERROR;
  }

  
  const std::string preview = readBuffer.substr(0, 400);
  RCLCPP_INFO(rclcpp::get_logger("RozumSystemHardware"),
              "HTTP %ld, payload %zu bytes, head: '%s%s'",
              http_code, readBuffer.size(), preview.c_str(),
              (readBuffer.size() > preview.size() ? "…":""));

  rapidjson::Document doc;
  doc.Parse(readBuffer.c_str());

  if (doc.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "RapidJSON parse error: %s (offset %zu). Head: '%s%s'",
                 rapidjson::GetParseError_En(doc.GetParseError()),
                 doc.GetErrorOffset(),
                 preview.c_str(),
                 (readBuffer.size() > preview.size() ? "…":""));
    return hardware_interface::return_type::ERROR;
  }

  if (!doc.HasMember("angles") || !doc["angles"].IsArray()) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "Invalid JSON (not array) from %s. Head: '%s%s'",
                 url.c_str(),
                 preview.c_str(),
                 (readBuffer.size() > preview.size() ? "…":""));
    return hardware_interface::return_type::ERROR;
  }

  
  int joint_id =0;
  if (doc.HasMember("angles") && doc["angles"].IsArray()) {
      const rapidjson::Value& arr = doc["angles"];
      for (auto& v : arr.GetArray()) {
          if (v.IsNumber()) {
              const double deg = v.GetDouble();
        joint_positions_[joint_id] = deg * PI / 180.0;
        std::string joint_name = "joint" + std::to_string(joint_id + 1);
        std::string name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
        set_state(name_pos, joint_positions_[joint_id]);
        joint_id++;
          }
      }
  }

  
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type RozumSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  
  const std::string url = "http://10.10.10.20:8081/pose?speed=10&motionType=joint";

  
  std::string json_payload;

  {
    json_payload = R"({"angles":[)";
    for (size_t i = 0; i < 6; ++i)
    {
      std::string joint_name = "joint" + std::to_string(i + 1);
      std::string name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
      const double deg = get_command(name_pos) * 180.0 / PI;
      if(std::isnan(deg)){
        return hardware_interface::return_type::OK;
      }
      json_payload += std::to_string(deg);
      if (i + 1 < 6) json_payload += ",";
    }
    json_payload += "]}";
  }

  CURL* curl = curl_easy_init();
  if (!curl) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"), "curl_easy_init() failed (write)");
    return hardware_interface::return_type::ERROR;
  }

  struct curl_slist* hdrs = nullptr;
  hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
  hdrs = curl_slist_append(hdrs, "Accept: application/json");

  char errbuf[CURL_ERROR_SIZE] = {0};
  std::string response;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_payload.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json_payload.size());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
  
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  CURLcode res = curl_easy_perform(curl);

  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_slist_free_all(hdrs);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "CURL write failed (%d): %s ; url=%s ; payload=%s",
                 static_cast<int>(res),
                 errbuf[0] ? errbuf : curl_easy_strerror(res),
                 url.c_str(), json_payload.c_str());
    return hardware_interface::return_type::ERROR;
  }

  if (http_code < 200 || http_code >= 300) {
    RCLCPP_ERROR(rclcpp::get_logger("RozumSystemHardware"),
                 "HTTP %ld on write to %s ; payload=%s ; resp_head='%s%s'",
                 http_code, url.c_str(), json_payload.c_str(),
                 response.substr(0, 200).c_str(),
                 (response.size() > 200 ? "…":""));
    return hardware_interface::return_type::ERROR;
  }

  
  return hardware_interface::return_type::OK;
}

}// namespac10joint_namee rozum_pulse_75_hardware

PLUGINLIB_EXPORT_CLASS(rozum_pulse_75_hardware::RozumSystemHardware,
                       hardware_interface::SystemInterface)
