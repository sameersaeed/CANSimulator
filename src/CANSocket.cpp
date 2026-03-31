#include <cstring>
#include <cerrno>
#include <cmath>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <sys/types.h>
#include <unistd.h>

#include <stdexcept>

#include "CANSocket.hpp"

CANSocket::CANSocket(const std::string& interface) : m_interface(interface) {
    m_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_fd < 0) throw std::runtime_error("socket(PF_CAN) failed: " + std::string(strerror(errno)));

    // resolve interface name to ifindex
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(m_fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(m_fd); m_fd = -1;

        throw std::runtime_error("Interface '" + interface + "' not found. "
            "Run: sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0");
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(m_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(m_fd); m_fd = -1;
        throw std::runtime_error("bind() failed: " + std::string(strerror(errno)));
    }
}

CANSocket::~CANSocket() {
    if (m_fd >= 0) ::close(m_fd);
}

bool CANSocket::sendFrame(const can_frame& frame) {
    ssize_t written = ::write(m_fd, &frame, sizeof(frame));
    return written == static_cast<ssize_t>(sizeof(frame));
}

std::optional<can_frame> CANSocket::receiveFrame(double timeoutS) {
    fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(m_fd, &readFds);

    struct timeval tv{};
    double intpart;
    double fracpart = std::modf(timeoutS, &intpart); // fractional secs
    tv.tv_sec  = static_cast<time_t>(intpart);
    tv.tv_usec = static_cast<suseconds_t>(fracpart * 1'000'000); // convert fractional secs to microsecs

    // handle timeoutS
    int ret = ::select(m_fd + 1, &readFds, nullptr, nullptr, &tv);
    if (ret <= 0) return std::nullopt;

    can_frame frame{};
    if (::read(m_fd, &frame, sizeof(frame)) < static_cast<ssize_t>(sizeof(frame)))
        return std::nullopt;

    return frame;
}

// only accept frames where (can_id & mask) == id
bool CANSocket::setFilter(canid_t id, canid_t mask) {
    struct can_filter filter[1];
    
    filter[0].can_id   = id;
    filter[0].can_mask = mask;

    return ::setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) == 0;
}
