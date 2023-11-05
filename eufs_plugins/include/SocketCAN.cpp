#include "SocketCAN.hpp"

#include <ifaddrs.h>
#include <linux/can.h>
#include <net/if.h>
#include <netdb.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

SocketCAN::SocketCAN() {
    this->isConnected = false;
    this->sock = -1;
}

bool SocketCAN::setup(std::string interface) {
    // create socket
    if (this->sock == -1) {
        this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->sock == -1) {
            std::cout << "CAN - Failed to create socket." << std::endl;
            return false;
        }
    }

    // get interface index
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface.c_str());
    ioctl(this->sock, SIOCGIFINDEX, &ifr);

    // bind socket to interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(this->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "CAN - Failed to bind interface to socket." << std::endl;
        return false;
    }

    this->isConnected = true;
    return true;
}

void SocketCAN::compose_socketcan_frame(CAN_MSG_Generic_t *msg, struct can_frame *frame) {
    // convert CAN_MSG_Generic_t to can_frame

    frame->can_id = msg->ID;
    if (msg->ID_TYPE) {
        // set extended bit
        frame->can_id |= CAN_EFF_FLAG;
    }

    frame->can_dlc = msg->DLC;
    for (uint8_t i = 0; i < msg->DLC; i++) {
        frame->data[i] = msg->data[i];
    }
}

bool SocketCAN::parse_socketcan_frame(struct can_frame *frame, CAN_MSG_Generic_t *msg) {
    msg->ID_TYPE = (frame->can_id & CAN_EFF_FLAG) != 0;

    if (msg->ID_TYPE != 0) {
        msg->ID = frame->can_id & CAN_EFF_MASK;
    } else {
        msg->ID = frame->can_id & CAN_SFF_MASK;
    }

    msg->DLC = frame->can_dlc;
    for (uint8_t i = 0; i < msg->DLC; i++) {
        msg->data[i] = frame->data[i];
    }

    return true;
}

void SocketCAN::tx(CAN_MSG_Generic_t *msg) {
    if (this->isConnected) {
        struct can_frame frame;
        compose_socketcan_frame(msg, &frame);

        if (write(this->sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            std::cout << "CAN - Failed TX." << std::endl;
        }
    }
}

std::shared_ptr<std::vector<CAN_MSG_Generic_t>> SocketCAN::rx() {
    auto msgs = std::make_shared<std::vector<CAN_MSG_Generic_t>>();
    CAN_MSG_Generic_t rxMsg;

    if (this->isConnected) {
        // only need to try Rx if we're connected
        // this function polls the socket for data, but will block if there's no data to recieve
        // can't know ahead of time how many messages to read, so will just read to oversized buffer

        // use DONTWAIT flag to make this non blocking
        ssize_t rxLen = recv(this->sock, this->rxBuf, SCAN_RECV_SIZE, MSG_DONTWAIT);

        if (rxLen > 0) {
            size_t len = rxLen;
            for (size_t offset = 0; offset < len; offset += sizeof(struct can_frame)) {
                if ((offset + sizeof(struct can_frame)) <= len) {
                    // there is a full frame here, read it

                    // convert appropriate bytes from socket into a can_frame
                    struct can_frame *frame = (struct can_frame *)&(this->rxBuf[offset]);
                    if (parse_socketcan_frame(frame, &rxMsg)) {
                        msgs->push_back(rxMsg);
                    }
                }
            }
        }
    }

    return msgs;
}

SocketCAN::~SocketCAN() {
    if (this->sock != -1) {
        close(this->sock);
        this->isConnected = false;
        this->sock = -1;
    }
}
