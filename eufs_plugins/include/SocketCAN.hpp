#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

// local include
#include "QUTMS_can.h"

const int SCAN_RECV_SIZE = 4096;

class SocketCAN {
   private:
    bool isConnected;
    int sock;
    uint8_t rxBuf[SCAN_RECV_SIZE];

   public:
    SocketCAN();

    bool setup(std::string interface);

    void tx(CAN_MSG_Generic_t *msg);
    std::shared_ptr<std::vector<CAN_MSG_Generic_t>> rx();

    void compose_socketcan_frame(CAN_MSG_Generic_t *msg, struct can_frame *frame);
    bool parse_socketcan_frame(struct can_frame *frame, CAN_MSG_Generic_t *msg);

    ~SocketCAN();
};
