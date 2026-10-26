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

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();
  bool headless = false;
  node->get_parameter_or("headless", headless, false);
  bool wait_for_effort_command = false;
  node->get_parameter_or("wait_for_effort_command", wait_for_effort_command, false);
  std::string initial_keyframe;
  node->get_parameter_or("initial_keyframe", initial_keyframe, std::string());

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);
  if (!initial_keyframe.empty())
  {
    const int key_id = mj_name2id(mujoco_model, mjOBJ_KEY, initial_keyframe.c_str());
    if (key_id < 0)
    {
      throw std::runtime_error("MuJoCo keyframe not found: " + initial_keyframe);
    }
    mj_resetDataKeyframe(mujoco_model, mujoco_data, key_id);
    RCLCPP_INFO(node->get_logger(), "Loaded MuJoCo keyframe '%s'", initial_keyframe.c_str());
  }

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  std::atomic<bool> first_command_received{false};
  std::atomic<bool> controller_ready{false};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_subscription;
  if (wait_for_effort_command)
  {
    command_subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands", 10,
      [&first_command_received](const std_msgs::msg::Float64MultiArray::SharedPtr message)
      {
        if (message->data.size() == 12 &&
            std::all_of(message->data.begin(), message->data.end(),
              [](double effort) { return std::isfinite(effort); }))
        {
          first_command_received = true;
        }
      });
    ready_subscription = node->create_subscription<std_msgs::msg::Bool>(
      "/go2/stand_ready", 10,
      [&controller_ready](const std_msgs::msg::Bool::SharedPtr message)
      {
        controller_ready = message->data;
      });
    RCLCPP_INFO(node->get_logger(), "Holding initial pose until effort controller and PD are ready.");
  }
  const auto wall_start = std::chrono::steady_clock::now();
  rclcpp::executors::SingleThreadedExecutor node_executor;
  node_executor.add_node(node);
  double release_time = -1.0;
  auto advance = [&]()
  {
    if (!rclcpp::ok()) return;
    node_executor.spin_some();
    if (!rclcpp::ok()) return;
    if (wait_for_effort_command && first_command_received && controller_ready &&
        release_time < 0.0)
    {
      release_time = mujoco_data->time + 0.1;
      RCLCPP_INFO(node->get_logger(), "First effort command received; releasing physics shortly.");
    }
    const bool run_physics = !wait_for_effort_command ||
      (release_time >= 0.0 && mujoco_data->time >= release_time);
    mujoco_control.update(run_physics);
    const auto target = wall_start +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(mujoco_data->time));
    std::this_thread::sleep_until(target);
  };

  if (headless)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Running MuJoCo in headless real-time mode.");
    while (rclcpp::ok())
    {
      advance();
    }

    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);
    return 0;
  }

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  cameras->init(mujoco_model);

  // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
  mjtNum last_cam_update = mujoco_data->time;
  while (rclcpp::ok() && !rendering->is_close_flag_raised())
  {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = mujoco_data->time;
    while (rclcpp::ok() && mujoco_data->time - simstart < 1.0 / 60.0)
    {
      advance();
    }
    if (!rclcpp::ok()) break;
    rendering->update();

    // Updating cameras at ~6 Hz
    // TODO(eholum): Break control and rendering into separate processes
    if (simstart - last_cam_update > 1.0 / 6.0)
    {
      cameras->update(mujoco_model, mujoco_data);
      last_cam_update = simstart;
    }
  }

  rendering->close();
  cameras->close();

  if (rclcpp::ok()) rclcpp::shutdown();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 0;
}
