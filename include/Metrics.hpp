#pragma once
#include <atomic>
#include <cstdint>

class Metrics {
public:
    void     recordRequest(int64_t latencyUs) noexcept;
    uint64_t requestCount() const noexcept;
    int64_t  maxLatencyUs() const noexcept;
    double   avgLatencyUs() const noexcept;
    void     reset()        noexcept;

private:
    std::atomic<uint64_t> m_requestCount   {0};
    std::atomic<int64_t>  m_totalLatencyUs{0};    // microseconds (us)
    std::atomic<int64_t>  m_maxLatencyUs  {0};
};
