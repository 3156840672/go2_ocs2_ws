// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include "mujoco_ros2_control/base_twist.hpp"
#ifdef GO2_RESOURCE_MANAGER_NEEDS_CONTEXT
#include "mujoco_ros2_control/mujoco_system.hpp"
#endif

#include <array>
#include <algorithm>
#include <stdexcept>

namespace mujoco_ros2_control {
MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr &node,
                                     mjModel *mujoco_model, mjData *mujoco_data)
    : node_(node), mj_model_(mujoco_model), mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() +
                                 std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME) {}

MujocoRos2Control::~MujocoRos2Control() {
  stop_cm_thread_ = true;
  if (cm_executor_ && rclcpp::ok()) {
    cm_executor_->cancel();
  }

  if (cm_thread_.joinable())
    cm_thread_.join();

  // controller_manager may detach itself while rclcpp is shutting down.
  if (cm_executor_ && controller_manager_ && rclcpp::ok()) {
    try {
      cm_executor_->remove_node(controller_manager_);
    } catch (const std::exception &error) {
      RCLCPP_DEBUG(logger_, "Controller manager already detached: %s", error.what());
    }
  }
}

std::string MujocoRos2Control::get_robot_description() {
  // Getting robot description from parameter first. If not set trying from
  // topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
      "robot_description_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  if (node->has_parameter("robot_description")) {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(logger_, "Failed to get robot_description from parameter. Will "
                       "listen on the ~/robot_description "
                       "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(),
      [&](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg->data.empty() && robot_description.empty())
          robot_description = msg->data;
      });

  while (robot_description.empty() && rclcpp::ok()) {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}

void MujocoRos2Control::init() {
  clock_publisher_ =
      node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  base_odometry_publisher_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/go2/base_odom", 10);
  foot_contacts_publisher_ =
      node_->create_publisher<go2_interfaces::msg::FootContacts>("/go2/foot_contacts", 10);

  const std::array<const char *, 4> foot_names{{"FL", "FR", "RL", "RR"}};
  for (size_t i = 0; i < foot_names.size(); ++i) {
    foot_geom_ids_[i] = mj_name2id(mj_model_, mjOBJ_GEOM, foot_names[i]);
    if (foot_geom_ids_[i] < 0) {
      throw std::runtime_error(std::string("Missing MuJoCo foot geom: ") + foot_names[i]);
    }
  }

  std::string base_body_name;
  node_->get_parameter_or("base_body_name", base_body_name,
                          std::string("base_link"));
  node_->get_parameter_or("odom_frame_id", odom_frame_id_,
                          std::string("world"));
  node_->get_parameter_or("base_frame_id", base_frame_id_,
                          std::string("trunk"));
  base_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, base_body_name.c_str());
  if (base_body_id_ < 0) {
    throw std::runtime_error("MuJoCo model does not contain base body: " +
                             base_body_name);
  }

  std::string urdf_string = this->get_robot_description();

  // setup actuators and mechanism control node.
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    control_hardware_info =
        hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error &ex) {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

#ifndef GO2_RESOURCE_MANAGER_NEEDS_CONTEXT
  try {
    robot_hw_sim_loader_.reset(
        new pluginlib::ClassLoader<MujocoSystemInterface>(
            "mujoco_ros2_control",
            "mujoco_ros2_control::MujocoSystemInterface"));
  } catch (pluginlib::LibraryLoadException &ex) {
    RCLCPP_ERROR_STREAM(
        logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
      std::make_unique<hardware_interface::ResourceManager>();

  try {
    resource_manager->load_urdf(urdf_string, false, false);
  } catch (...) {
    RCLCPP_ERROR(logger_, "Error while initializing URDF!");
  }

  for (const auto &hardware : control_hardware_info) {
    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
          robot_hw_sim_loader_->createUnmanagedInstance(
              robot_hw_sim_type_str_));
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR_STREAM(logger_,
                          "The plugin failed to load. Error: " << ex.what());
      continue;
    }

    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    if (!mujoco_system->init_sim(mj_model_, mj_data_, urdf_model, hardware)) {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return;
    }

    resource_manager->import_component(std::move(mujoco_system), hardware);

    rclcpp_lifecycle::State state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }
#else
  MujocoSystem::set_simulation_context(mj_model_, mj_data_, urdf_string);
#endif

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
#ifdef GO2_RESOURCE_MANAGER_NEEDS_CONTEXT
  auto cm_options = controller_manager::get_cm_node_options();
  cm_options.append_parameter_override("use_sim_time", true);
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      cm_executor_, urdf_string, true, "controller_manager", node_->get_namespace(),
      cm_options);
#else
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resource_manager), cm_executor_, "controller_manager",
      node_->get_namespace());
#endif
  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(
        logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ =
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 /
                                        static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
      rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]() {
    while (rclcpp::ok() && !stop_cm_thread_) {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);
}

void MujocoRos2Control::update(bool advance_physics) {
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec =
      static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);

  if (advance_physics) {
    mj_step1(mj_model_, mj_data_);
  } else {
    mj_forward(mj_model_, mj_data_);
  }

  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    publish_base_odometry(sim_time_ros);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // use same time as for read and update call - this is how it is done in
  // ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  if (advance_physics) {
    mj_step2(mj_model_, mj_data_);
  } else {
    // Keep ROS simulation time advancing so controllers can start without gravity.
    mj_data_->time += mj_model_->opt.timestep;
  }

  publish_foot_contacts(sim_time_ros);
}

void MujocoRos2Control::publish_foot_contacts(rclcpp::Time sim_time) {
  go2_interfaces::msg::FootContacts message;
  message.header.stamp = sim_time;
  message.header.frame_id = odom_frame_id_;
  message.foot_names = {{"FL", "FR", "RL", "RR"}};

  for (int contact_id = 0; contact_id < mj_data_->ncon; ++contact_id) {
    const auto &contact = mj_data_->contact[contact_id];
    if (contact.efc_address < 0) {
      continue;
    }
    for (size_t foot = 0; foot < foot_geom_ids_.size(); ++foot) {
      const int other_geom = contact.geom1 == foot_geom_ids_[foot] ? contact.geom2 :
          (contact.geom2 == foot_geom_ids_[foot] ? contact.geom1 : -1);
      if (other_geom < 0 || mj_model_->geom_bodyid[other_geom] != 0) {
        continue;
      }
      mjtNum wrench[6]{};
      mj_contactForce(mj_model_, mj_data_, contact_id, wrench);
      message.normal_force[foot] += std::max(0.0, static_cast<double>(wrench[0]));
    }
  }

  for (size_t foot = 0; foot < foot_geom_ids_.size(); ++foot) {
    message.in_contact[foot] = message.normal_force[foot] > 1.0;
  }
  foot_contacts_publisher_->publish(message);
}

void MujocoRos2Control::publish_base_odometry(rclcpp::Time sim_time) {
  nav_msgs::msg::Odometry message;
  message.header.stamp = sim_time;
  message.header.frame_id = odom_frame_id_;
  message.child_frame_id = base_frame_id_;

  const mjtNum *position = mj_data_->xpos + 3 * base_body_id_;
  const mjtNum *quaternion = mj_data_->xquat + 4 * base_body_id_;
  message.pose.pose.position.x = position[0];
  message.pose.pose.position.y = position[1];
  message.pose.pose.position.z = position[2];
  message.pose.pose.orientation.w = quaternion[0];
  message.pose.pose.orientation.x = quaternion[1];
  message.pose.pose.orientation.y = quaternion[2];
  message.pose.pose.orientation.z = quaternion[3];

  mjtNum local_velocity[6];
  bodyOriginLocalVelocity(mj_model_, mj_data_, base_body_id_, local_velocity);
  message.twist.twist.angular.x = local_velocity[0];
  message.twist.twist.angular.y = local_velocity[1];
  message.twist.twist.angular.z = local_velocity[2];
  message.twist.twist.linear.x = local_velocity[3];
  message.twist.twist.linear.y = local_velocity[4];
  message.twist.twist.linear.z = local_velocity[5];

  base_odometry_publisher_->publish(message);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time) {
  // TODO(sangteak601)
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

} // namespace mujoco_ros2_control
