/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file voice_command_listener.cpp
    * @brief ROS2 voice command subscriber implementation.
    */

#include "voice_command_listener.h"

#include <iostream>
#include <utility>

namespace perceptive_grasp {

namespace {

std::string TrimAscii(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

}  // namespace

VoiceCommandListener::VoiceCommandListener(std::string node_name,
                                            std::string input_topic,
                                            CommandCallback callback,
                                            std::string status_topic)
    : node_name_(std::move(node_name)),
        input_topic_(std::move(input_topic)),
        status_topic_(std::move(status_topic)),
        callback_(std::move(callback)) {}

VoiceCommandListener::~VoiceCommandListener() { Stop(); }

bool VoiceCommandListener::Start() {
    if (running_.load()) return true;
    if (input_topic_.empty() && status_topic_.empty()) {
        std::cerr << "[VoiceROS] no input or status topic configured"
                    << std::endl;
        return false;
    }
    if (!input_topic_.empty() && !callback_) {
        std::cerr << "[VoiceROS] command callback is empty" << std::endl;
        return false;
    }

    context_ = std::make_shared<rclcpp::Context>();
    int argc = 0;
    char** argv = nullptr;
    context_->init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.context(context_);
    node_ = std::make_shared<rclcpp::Node>(node_name_, node_options);
    if (!input_topic_.empty()) {
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            input_topic_, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                OnMessage(msg);
            });
    }
    if (!status_topic_.empty()) {
        pub_ = node_->create_publisher<std_msgs::msg::String>(
            status_topic_, 10);
    }

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        executor_options);
    executor_->add_node(node_);

    running_.store(true);
    spin_thread_ = std::thread([this]() {
        if (!input_topic_.empty()) {
            std::cout << "[VoiceROS] Listening on topic: " << input_topic_
                        << std::endl;
        }
        if (!status_topic_.empty()) {
            std::cout << "[VoiceROS] Publishing status on topic: "
                        << status_topic_ << std::endl;
        }
        executor_->spin();
    });
    return true;
}

void VoiceCommandListener::Stop() {
    if (!running_.exchange(false)) return;
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    if (executor_ && node_) {
        executor_->remove_node(node_->get_node_base_interface());
    }
    {
        std::lock_guard<std::mutex> lock(pub_mutex_);
        pub_.reset();
    }
    sub_.reset();
    node_.reset();
    executor_.reset();
    if (context_ && context_->is_valid()) {
        context_->shutdown("voice listener stopped");
    }
    context_.reset();
}

void VoiceCommandListener::PublishStatus(const std::string& text) {
    if (text.empty()) return;
    std::lock_guard<std::mutex> lock(pub_mutex_);
    if (!pub_) return;
    std_msgs::msg::String out;
    out.data = text;
    pub_->publish(out);
}

void VoiceCommandListener::OnMessage(
    const std_msgs::msg::String::SharedPtr msg) {
    if (!msg) return;
    std::string text = TrimAscii(msg->data);
    if (text.empty()) return;

    std::cout << "[VoiceROS] Received: " << text << std::endl;
    callback_(text);
}

}  // namespace perceptive_grasp
