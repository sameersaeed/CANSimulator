#include "metrics.hpp"

void Metrics::record_request(int64_t latency_us) noexcept {
    ++m_request_count;
    m_total_latency_us += latency_us;

    // update max 
    int64_t cur = m_max_latency_us.load(std::memory_order_relaxed);
    while (latency_us > cur &&
           !m_max_latency_us.compare_exchange_weak(
               cur, latency_us,
               std::memory_order_release,
               std::memory_order_relaxed)) {}
}

uint64_t Metrics::request_count()  const noexcept { return m_request_count.load(); }
int64_t  Metrics::max_latency_us() const noexcept { return m_max_latency_us.load(); }

double Metrics::avg_latency_us() const noexcept {
    auto count = m_request_count.load();
    return count ? static_cast<double>(m_total_latency_us.load()) / count : 0.0;
}

void Metrics::reset() noexcept {
    m_request_count    = 0;
    m_total_latency_us = 0;
    m_max_latency_us   = 0;
}
