#include "plaguebot_firmware/plaguebot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace plaguebot_firmware
{
PlaguebotInterface::PlaguebotInterface() {}

PlaguebotInterface::~PlaguebotInterface()
{
  if (esp32_.IsOpen()) {
    try { esp32_.Close(); }
    catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("PlaguebotInterface"),
        "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn PlaguebotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) return result;
  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range &e) {
    RCLCPP_FATAL(rclcpp::get_logger("PlaguebotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }
  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  last_run_ = rclcpp::Clock().now();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PlaguebotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PlaguebotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn PlaguebotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PlaguebotInterface"), "Starting Plague-Bot hardware ...");
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };
  try {
    esp32_.Open(port_);
    esp32_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("PlaguebotInterface"),
        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("PlaguebotInterface"), "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlaguebotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PlaguebotInterface"), "Stopping Plague-Bot hardware ...");
  if (esp32_.IsOpen()) {
    try { esp32_.Close(); }
    catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("PlaguebotInterface"),
          "Something went wrong while closing connection with port " << port_);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("PlaguebotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type PlaguebotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (esp32_.IsDataAvailable()) {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    try { esp32_.ReadLine(message); }
    catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("PlaguebotInterface"), "Error reading from serial");
      return hardware_interface::return_type::OK;
    }
    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;
    while (std::getline(ss, res, ',')) {
      if (res.size() < 3) continue;
      multiplier = res.at(1) == 'p' ? 1 : -1;
      if (res.at(0) == 'r') {
        velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      } else if (res.at(0) == 'l') {
        velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PlaguebotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
  std::string czr = std::abs(velocity_commands_.at(0)) < 10.0 ? "0" : "";
  std::string czl = std::abs(velocity_commands_.at(1)) < 10.0 ? "0" : "";
  message_stream << std::fixed << std::setprecision(2)
    << "r" << right_wheel_sign << czr << std::abs(velocity_commands_.at(0))
    << ",l" << left_wheel_sign << czl << std::abs(velocity_commands_.at(1)) << ",";
  try { esp32_.Write(message_stream.str()); }
  catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("PlaguebotInterface"),
        "Something went wrong while sending the message "
            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
}

PLUGINLIB_EXPORT_CLASS(plaguebot_firmware::PlaguebotInterface, hardware_interface::SystemInterface)
