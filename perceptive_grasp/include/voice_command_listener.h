/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file voice_command_listener.h
    * @brief ROS2 voice command subscriber for perceptive grasp pipeline.
    */

#ifndef VOICE_COMMAND_LISTENER_H
#define VOICE_COMMAND_LISTENER_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>

namespace perceptive_grasp {

class VoiceCommandListener {
public:
    using CommandCallback = std::function<void(const std::string&)>;

    VoiceCommandListener(std::string node_name,
                        std::string input_topic,
                        CommandCallback callback,
                        std::string status_topic = "");
    ~VoiceCommandListener();

    bool Start();
    void Stop();
    void PublishStatus(const std::string& text);

private:
    std::string node_name_;
    std::string input_topic_;
    std::string status_topic_;
    CommandCallback callback_;

    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    std::thread spin_thread_;
    std::atomic<bool> running_{false};
    std::mutex pub_mutex_;

    void OnMessage(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace perceptive_grasp

#endif  // VOICE_COMMAND_LISTENER_H
