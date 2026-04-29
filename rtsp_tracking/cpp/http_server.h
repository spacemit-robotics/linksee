/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <memory>
#include <string>
#include <thread>

class Pipeline;

class HttpServer {
public:
    HttpServer(Pipeline* pipeline, int port,
        const std::string& web_root = "");
    ~HttpServer();

    HttpServer(const HttpServer&) = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    void Start();
    void Stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif  // HTTP_SERVER_H
