/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <limits.h>
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <string>

#include "linksee_app/runner.hpp"

namespace linksee_app {

namespace {

constexpr int kSuccessExitCode = 0;

std::string GetSourceDirectory() {
    const std::string file_path = __FILE__;
    const std::string::size_type pos = file_path.find_last_of('/');
    if (pos == std::string::npos) {
        return ".";
    }
    return file_path.substr(0, pos);
}

std::string CanonicalPath(const std::string& path) {
    char resolved_path[PATH_MAX] = {0};
    if (realpath(path.c_str(), resolved_path) == nullptr) {
        return path;
    }
    return resolved_path;
}

std::string JoinPath(const std::string& left, const std::string& right) {
    if (left.empty() || left[left.size() - 1] == '/') {
        return left + right;
    }
    return left + "/" + right;
}

}  // namespace

Runner::Runner() = default;

std::string Runner::app_dir() const {
    return CanonicalPath(JoinPath(GetSourceDirectory(), "../.."));
}

std::string Runner::repo_root() const {
    return CanonicalPath(JoinPath(app_dir(), "../../../.."));
}

std::string Runner::venv_dir() const {
    return JoinPath(repo_root(), "output/envs/linksee_app");
}

std::string Runner::setup_script() const {
    return JoinPath(app_dir(), "scripts/setup_env.sh");
}

std::string Runner::host_start_script() const {
    return JoinPath(app_dir(), "scripts/start_host.sh");
}

std::string Runner::inference_start_script() const {
    return JoinPath(app_dir(), "scripts/start_inference.sh");
}

std::string Runner::host_stop_script() const {
    return JoinPath(app_dir(), "scripts/stop_host.sh");
}

std::string Runner::inference_stop_script() const {
    return JoinPath(app_dir(), "scripts/stop_inference.sh");
}

bool Runner::EnsureEnvironment() const {
    if (access(venv_dir().c_str(), F_OK) == 0) {
        return true;
    }

    const std::string command = "bash " + Quote(setup_script());
    return RunShellCommand(command) == kSuccessExitCode;
}

CommandResult Runner::RunStartHost() const {
    return RunScript(host_start_script());
}

CommandResult Runner::RunStartInference() const {
    return RunScript(inference_start_script());
}

CommandResult Runner::RunStopHost() const {
    return RunScript(host_stop_script());
}

CommandResult Runner::RunStopInference() const {
    return RunScript(inference_stop_script());
}

CommandResult Runner::RunScript(const std::string& script_path) const {
    CommandResult result;
    std::ostringstream command;
    command << "bash " << Quote(script_path);
    result.command = command.str();
    result.exit_code = RunShellCommand(result.command);
    return result;
}

std::string Runner::Quote(const std::string& value) {
    std::string escaped = "'";
    for (char ch : value) {
        if (ch == '\'') {
            escaped += "'\\''";
        } else {
            escaped += ch;
        }
    }
    escaped += "'";
    return escaped;
}

int Runner::RunShellCommand(const std::string& command) {
    return system(command.c_str());
}

}  // namespace linksee_app
