//
// Created by tlab-uav on 24-9-13.
//

#include "joystick_input/JoystickInput.h"

#include <cmath>

using std::placeholders::_1;

JoystickInput::JoystickInput() : Node("joysick_input_node") {
    deadzone_ = declare_parameter<double>("deadzone", 0.08);
    publisher_ = create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickInput::joy_callback, this, _1));
}

float JoystickInput::axis(const sensor_msgs::msg::Joy& msg, const size_t index) const {
    if (index >= msg.axes.size()) return 0.0F;
    return std::abs(msg.axes[index]) < deadzone_ ? 0.0F : msg.axes[index];
}

bool JoystickInput::button(const sensor_msgs::msg::Joy& msg, const size_t index) const {
    return index < msg.buttons.size() && msg.buttons[index] != 0;
}

void JoystickInput::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    // Thunderobot G30S TE mapping reported by the Linux joy driver.
    // buttons: A=0, B=1, X=3, Y=4, LB=6, RB=7, LT=8, RT=9, back=10, start=11
    // axes: left X/Y=0/1, right X/Y=2/3, RT=4, LT=5
    const float lx = -axis(*msg, 0);
    const float ly = axis(*msg, 1);
    const float rx = -axis(*msg, 2);
    const float ry = axis(*msg, 3);

    // This device can report all stick axes as 1.0 while it initializes. Do not
    // forward motion until every stick has been observed near its neutral point.
    constexpr float arm_threshold = 0.25F;
    if (!motion_armed_ && std::abs(lx) <= arm_threshold && std::abs(ly) <= arm_threshold &&
        std::abs(rx) <= arm_threshold && std::abs(ry) <= arm_threshold) {
        motion_armed_ = true;
        RCLCPP_INFO(get_logger(), "Joystick motion enabled after sticks returned to neutral.");
    }
    inputs_.lx = motion_armed_ ? lx : 0.0F;
    inputs_.ly = motion_armed_ ? ly : 0.0F;
    inputs_.rx = motion_armed_ ? rx : 0.0F;
    inputs_.ry = motion_armed_ ? ry : 0.0F;

    const bool lt_pressed = button(*msg, 8) || (msg->axes.size() > 5 && msg->axes[5] < 0.5F);

    if (button(*msg, 1) && button(*msg, 6)) {
        inputs_.command = 1; // LB + B
    } else if (button(*msg, 0) && button(*msg, 6)) {
        inputs_.command = 2; // LB + A
    } else if (button(*msg, 3) && button(*msg, 6)) {
        inputs_.command = 3; // LB + X
    } else if (button(*msg, 4) && button(*msg, 6)) {
        inputs_.command = 4; // LB + Y
    } else if (lt_pressed && button(*msg, 1)) {
        inputs_.command = 5; // LT + B
    } else if (lt_pressed && button(*msg, 0)) {
        inputs_.command = 6; // LT + A
    } else if (lt_pressed && button(*msg, 3)) {
        inputs_.command = 7; // LT + X
    } else if (lt_pressed && button(*msg, 4)) {
        inputs_.command = 8; // LT + Y
    } else if (button(*msg, 11)) {
        inputs_.command = 9; // START
    } else {
        inputs_.command = 0;
    }
    publisher_->publish(inputs_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
