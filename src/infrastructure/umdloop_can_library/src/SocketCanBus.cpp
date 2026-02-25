#include "umdloop_can_library/SocketCanBus.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>

namespace CANLib {

namespace {
    void mapCanFrameToSocketCan(const CanFrame& frame, ::can_frame& can_frame) {
        std::memset(&can_frame, 0, sizeof(can_frame));
        can_frame.can_id = frame.id;
        can_frame.can_dlc = frame.dlc;
        
        if (frame.is_extended) {
            can_frame.can_id |= CAN_EFF_FLAG;
        }
        if (frame.is_rtr) {
            can_frame.can_id |= CAN_RTR_FLAG;
        }
        
        std::copy(frame.data.begin(), frame.data.begin() + frame.dlc, can_frame.data);
    }

    void mapSocketCanToCanFrame(const ::can_frame& can_frame, CanFrame& frame) {
        frame.id = can_frame.can_id & CAN_ERR_MASK;
        frame.dlc = std::min(can_frame.can_dlc, static_cast<__u8>(8));
        frame.is_extended = (can_frame.can_id & CAN_EFF_FLAG) != 0;
        frame.is_rtr = (can_frame.can_id & CAN_RTR_FLAG) != 0;
        frame.data.fill(0);
        std::copy(can_frame.data, can_frame.data + frame.dlc, frame.data.begin());
    }
}

SocketCanBus::SocketCanBus() : socketFd_(-1), running_(false) {}

SocketCanBus::~SocketCanBus() {
    close();
}

bool SocketCanBus::open(const std::string& interface_name, ReceiveCallback callback) {
    if (running_) return false;

    receiveCallback_ = callback;
    interfaceName_ = interface_name;

    // Create socket
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketFd_ < 0) {
        std::cerr << "Failed to create SocketCAN socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_name.c_str());
    if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << interface_name 
                  << ": " << strerror(errno) << std::endl;
        ::close(socketFd_);
        socketFd_ = -1;
        return false;
    }

    // Bind socket to CAN interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketFd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket to " << interface_name 
                  << ": " << strerror(errno) << std::endl;
        ::close(socketFd_);
        socketFd_ = -1;
        return false;
    }

    // Enable receipt of own messages (loopback)
    int loopback = 1;
    setsockopt(socketFd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
               &loopback, sizeof(loopback));

    // Enable receipt of CAN FD frames
    int canfd_on = 1;
    setsockopt(socketFd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
               &canfd_on, sizeof(canfd_on));

    running_ = true;
    receiveThread_ = std::thread(&SocketCanBus::receiveLoop_, this);
    return true;
}

void SocketCanBus::close() {
    if (running_) {
        running_ = false;
        if (receiveThread_.joinable()) {
            receiveThread_.join();
        }
        if (socketFd_ >= 0) {
            ::close(socketFd_);
            socketFd_ = -1;
        }
    }
}

bool SocketCanBus::send(const CanFrame& frame) {
    if (!running_ || socketFd_ < 0) return false;

    ::can_frame can_frame;
    mapCanFrameToSocketCan(frame, can_frame);
    
    ssize_t nbytes = write(socketFd_, &can_frame, sizeof(can_frame));

    //static int send_count = 0;
    //static auto last_time = std::chrono::steady_clock::now();
    //send_count++;
    //if (send_count % 100 == 0) {
    //    auto now = std::chrono::steady_clock::now();
    //    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    //    std::cout << "CAN: messages sent: " << send_count << " | Time for last 100: " << duration << " ms" << std::endl;
    //    last_time = now;
    //}

    if (nbytes != sizeof(can_frame)) {
        std::cerr << "CAN: Failed to send frame - ID: 0x" << std::hex << frame.id
                  << std::dec << ", DLC: " << static_cast<int>(frame.dlc)
                  << ", Data: [";
        for (int i = 0; i < frame.dlc; i++) {
            std::cerr << std::hex << static_cast<int>(frame.data[i]);
            if (i < frame.dlc - 1) std::cerr << " ";
        }
        std::cerr << std::dec << "] - Error: " << strerror(errno)
                  << " (bytes written: " << nbytes << ")" << std::endl;
        return false;
    }

    return true;
}

void SocketCanBus::receiveLoop_() {
    ::can_frame can_frame;
    
    while (running_) {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(socketFd_, &readfds);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms timeout
        
        int result = select(socketFd_ + 1, &readfds, NULL, NULL, &timeout);
        
        if (result < 0) {
            if (errno != EINTR) {
                std::cerr << "SocketCAN select error: " << strerror(errno) << std::endl;
                break;
            }
            continue;
        }
        
        if (result == 0) {
            // Timeout, continue loop
            continue;
        }
        
        if (FD_ISSET(socketFd_, &readfds)) {
            ssize_t nbytes = read(socketFd_, &can_frame, sizeof(can_frame));
            
            if (nbytes == sizeof(can_frame) && receiveCallback_) {
                CanFrame frame;
                mapSocketCanToCanFrame(can_frame, frame);
                receiveCallback_(frame);
            } else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "SocketCAN read error: " << strerror(errno) << std::endl;
                break;
            }
        }
    }
}

} // namespace CANLib