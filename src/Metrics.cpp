#include "Metrics.hpp"

// recordRequest takes a latency value in microseconds (us)
void Metrics::recordRequest(int64_t latencyUs) noexcept {
    ++m_requestCount;
    m_totalLatencyUs += latencyUs;

    // update max 
    int64_t cur = m_maxLatencyUs.load(std::memory_order_relaxed);
    while (latencyUs > cur && !m_maxLatencyUs.compare_exchange_weak(
        cur, latencyUs, 
        std::memory_order_release, std::memory_order_relaxed)
    ) {}
}

uint64_t Metrics::requestCount()  const noexcept { return m_requestCount.load(); }
int64_t  Metrics::maxLatencyUs() const noexcept { return m_maxLatencyUs.load(); }

double Metrics::avgLatencyUs() const noexcept {
    auto count = m_requestCount.load();
    return count ? static_cast<double>(m_totalLatencyUs.load()) / count : 0.0;
}

void Metrics::reset() noexcept {
    m_requestCount    = 0;
    m_totalLatencyUs = 0;
    m_maxLatencyUs   = 0;
}
