#pragma once
#include <atomic>
#include <cstdint>

class Metrics {
public:
    void     record_request(int64_t latency_us) noexcept;
    uint64_t request_count()  const noexcept;
    int64_t  max_latency_us() const noexcept;
    double   avg_latency_us() const noexcept;
    void     reset()          noexcept;

private:
    std::atomic<uint64_t> m_request_count   {0};
    std::atomic<int64_t>  m_total_latency_us{0};
    std::atomic<int64_t>  m_max_latency_us  {0};
};
