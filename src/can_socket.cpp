#include "can_socket.hpp"

#include <cstring>
#include <cerrno>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <unistd.h>

#include <stdexcept>

CANSocket::CANSocket(const std::string& interface) : m_interface(interface) {
    m_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_fd < 0)
        throw std::runtime_error("socket(PF_CAN) failed: " + std::string(strerror(errno)));

    // resolve interface name to ifindex
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(m_fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(m_fd); m_fd = -1;
        throw std::runtime_error("Interface '" + interface + "' not found. "
                                 "Run: sudo modprobe vcan && "
                                 "sudo ip link add dev vcan0 type vcan && "
                                 "sudo ip link set up vcan0");
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

bool CANSocket::send_frame(const can_frame& frame) {
    ssize_t written = ::write(m_fd, &frame, sizeof(frame));
    return written == static_cast<ssize_t>(sizeof(frame));
}

std::optional<can_frame> CANSocket::receive_frame(int timeout_ms) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(m_fd, &read_fds);

    struct timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    // handle timeout
    int ret = ::select(m_fd + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret <= 0) return std::nullopt;

    can_frame frame{};
    if (::read(m_fd, &frame, sizeof(frame)) < static_cast<ssize_t>(sizeof(frame)))
        return std::nullopt;

    return frame;
}

// only accept frames where (can_id & mask) == id
bool CANSocket::set_filter(canid_t id, canid_t mask) {
    struct can_filter filter[1];
    
    filter[0].can_id   = id;
    filter[0].can_mask = mask;

    return ::setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) == 0;
}
