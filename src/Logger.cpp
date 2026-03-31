#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>

#include "Logger.hpp"

void Logger::info (const std::string& msg) { log("INFO ", msg); }
void Logger::warn (const std::string& msg) { log("WARN ", msg); }
void Logger::error(const std::string& msg) { log("ERROR", msg); }

void Logger::log(const char* level, const std::string& msg) {
    static std::mutex mtx;

    std::chrono::time_point now = std::chrono::system_clock::now();
    std::time_t t   = std::chrono::system_clock::to_time_t(now);
    std::lock_guard<std::mutex> lk(mtx);

    std::cout << "[" << std::put_time(std::localtime(&t), "%H:%M:%S") << "] " << 
        "[" << level << "] " << msg << "\n";
}
