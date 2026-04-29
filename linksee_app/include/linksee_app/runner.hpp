/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RUNNER_HPP
#define RUNNER_HPP

#include <string>

namespace linksee_app {

struct CommandResult {
    int exit_code = 0;
    std::string command;
};

class Runner {
public:
    Runner();

    std::string app_dir() const;
    std::string repo_root() const;
    std::string venv_dir() const;
    std::string setup_script() const;
    std::string host_start_script() const;
    std::string inference_start_script() const;
    std::string host_stop_script() const;
    std::string inference_stop_script() const;

    bool EnsureEnvironment() const;
    CommandResult RunStartHost() const;
    CommandResult RunStartInference() const;
    CommandResult RunStopHost() const;
    CommandResult RunStopInference() const;

private:
    static std::string Quote(const std::string& value);
    static int RunShellCommand(const std::string& command);
    CommandResult RunScript(const std::string& script_path) const;
};

}  // namespace linksee_app

#endif  // RUNNER_HPP
