#include <algorithm>
#include <chrono>
#include <iomanip>
#include <ios>
#include <iostream>
#include <numeric>
#include <vector>

#include <linux/can.h>

#include "OBDClient.hpp"

// only receive ECU 1 response frames (0x7E8), not sent requests
OBDClient::OBDClient(const std::string& iface) : m_socket(iface) { 
    m_socket.setFilter(0x7E8, 0x7FF);
}

std::optional<QueryResult> OBDClient::query(PID pid, double timeoutS) {
    can_frame request{};
    request.can_id  = 0x7DF; // OBD-II functional broadcast
    request.can_dlc = 8;
    request.data[0] = 0x02; // 2 additional bytes
    request.data[1] = 0x01; // Mode 01, current data
    request.data[2] = static_cast<uint8_t>(pid);

    auto start = std::chrono::steady_clock::now();

    if (!m_socket.sendFrame(request)) return std::nullopt;

    // discard stale frames until a matching response is found
    while (true) {
        auto frame = m_socket.receiveFrame(timeoutS);
        if (!frame) return std::nullopt;

        if (frame->can_id  != 0x7E8)                     continue;
        if (frame->can_dlc <  4)                         continue;
        if (frame->data[1] != 0x41)                      continue;
        if (frame->data[2] != static_cast<uint8_t>(pid)) continue;

        auto latencyUs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();

        auto decoded = decodeResponse(frame->data[2], frame->data[3], frame->data[4]);
        if (!decoded) return std::nullopt;

        return QueryResult{decoded->name, decoded->value, decoded->unit, latencyUs};
    }
}

void OBDClient::benchmark(PID pid, int count) {
    std::vector<int64_t> latencies;
    latencies.reserve(count);
    int success = 0;

    for (int i = 0; i < count; ++i) {
        auto result = query(pid, 5); // 5 sec timeout
        if (result) {
            ++success;
            latencies.push_back(result->latencyUs);
        }
    }

    if (latencies.empty()) {
        std::cout << "No successful queries... is the simulator running?\n";
        return;
    }

    std::sort(latencies.begin(), latencies.end());

    double avg = static_cast<double>(std::accumulate(latencies.begin(), latencies.end(), int64_t{0})) / latencies.size();
    double total    = std::accumulate(latencies.begin(), latencies.end(), int64_t{0}) / 1e6;
    double throughput = (total > 0) ? success / total : 0.0; 

    auto percentile = [&](int p) -> double {
        size_t idx = std::min(static_cast<size_t>(latencies.size() * p / 100), latencies.size() - 1);

        return latencies[idx] / 1000.0;
    };

    std::cout << "\n=== CANSimulator Benchmark: " << pidToName(pid) << " x" << count << " ===\n" << std::fixed << std::setprecision(3) << 
        "  Successful:   " << success << "/" << count << "\n" << 
        "  Avg latencyUs:  " << avg / 1000.0 << " ms\n" << 
        "  p50 latencyUs:  " << percentile(50)  << " ms\n" << 
        "  p95 latencyUs:  " << percentile(95)  << " ms\n" << 
        "  p99 latencyUs:  " << percentile(99)  << " ms\n" << 
        "  Max latencyUs:  " << latencies.back() / 1000.0 << " ms\n" << 
        "  Throughput:   " << std::setprecision(0) << throughput << " req/s\n\n";
}
