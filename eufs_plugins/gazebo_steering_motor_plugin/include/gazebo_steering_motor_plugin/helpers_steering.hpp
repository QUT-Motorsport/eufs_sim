
#include <stddef.h>
#include <stdint.h>

#include <map>
#include <string>

#include "driverless_msgs/msg/can.hpp"

#define C5E_NODE_ID 0x70
#define C5E_TX_BOOT_UP_ID 0x700 + C5E_NODE_ID
#define C5E_TX_EMCY_ID 0x80 + C5E_NODE_ID
#define C5E_TX_STATUS_ID 0x180 + C5E_NODE_ID
#define C5E_RX_STATUS_ID 0x200 + C5E_NODE_ID
#define C5E_TX_POS_ID 0x280 + C5E_NODE_ID
#define C5E_TX_SRV_ID 0x580 + C5E_NODE_ID
#define C5E_RX_SRV_ID 0x600 + C5E_NODE_ID

std::map<uint8_t, size_t> can_open_size_map = {
    {0x2F, 1},
    {0x2B, 2},
    {0x27, 3},
    {0x23, 4},
};

typedef enum c5e_object_id {
    HOME_OFFSET = 0x607C,
    MOTION_PROFILE_TYPE = 0x6086,
    PROFILE_VELOCITY = 0x6081,
    END_VELOCITY = 0x6082,
    PROFILE_ACCELERATION = 0x6083,
    PROFILE_DECELERATION = 0x6084,
    QUICK_STOP_DECELERATION = 0x6085,
    MAX_ACCELERATION = 0x60C5,
    MAX_DECELERATION = 0x60C6,
    MODE_OF_OPERATION = 0x6060,
    TARGET_POSITION = 0x607A,
    CONTROL_WORD = 0x6040,
    STATUS_WORD = 0x6041,
    POSITION_ACTUAL_VAL = 0x6064,
    ERROR_REGISTER = 0x1001,
} c5e_object_id_t;

// name enum for objects
std::map<uint16_t, std::string> c5e_object_names = {
    {HOME_OFFSET, "Home offset"},
    {MOTION_PROFILE_TYPE, "Motion profile type"},
    {PROFILE_VELOCITY, "Profile velocity"},
    {END_VELOCITY, "End velocity"},
    {PROFILE_ACCELERATION, "Profile acceleration"},
    {PROFILE_DECELERATION, "Profile deceleration"},
    {QUICK_STOP_DECELERATION, "Quick stop deceleration"},
    {MAX_ACCELERATION, "Max acceleration"},
    {MAX_DECELERATION, "Max deceleration"},
    {MODE_OF_OPERATION, "Mode of operation"},
    {TARGET_POSITION, "Target position"},
    {CONTROL_WORD, "Control word"},
    {STATUS_WORD, "Status word"},
    {POSITION_ACTUAL_VAL, "Position actual value"},
    {ERROR_REGISTER, "Error register"},
};

struct c5e_state {
    std::string name;
    uint16_t mask;
    uint16_t state_id;
    uint16_t control_word;
    uint8_t step;

    bool operator==(const c5e_state &rhs) {
        return (this->name == rhs.name && this->mask == rhs.mask && this->state_id == rhs.state_id);
    }

    bool operator!=(const c5e_state &rhs) { return !(*this == rhs); }
};

typedef enum c5e_state_id {
    NRTSO = 0b0000000000000000,  // Not ready to switch on
    SOD = 0b0000000001000000,    // Switch on disabled
    RTSO = 0b0000000000100001,   // Ready to switch on
    SO = 0b0000000000100011,     // Switched on
    OE = 0b0000000000100111,     // Operation enabled
    QSA = 0b0000000000000111,    // Quick stop active
    FRA = 0b0000000000001111,    // Fault reaction active
    F = 0b0000000000001000       // Fault
} c5e_state_id_t;

// State Map Definitions
std::map<uint16_t, c5e_state> states = {
    {NRTSO, {"Not ready to switch on", 0b0000000001001111, NRTSO, 0b0000, 0}},
    {SOD, {"Switch on disabled", 0b0000000001001111, SOD, 0b0000, 1}},
    {RTSO, {"Ready to switch on", 0b0000000001101111, RTSO, 0b0110, 2}},
    {SO, {"Switched on", 0b0000000001101111, SO, 0b0111, 3}},
    {OE, {"Operation enabled", 0b0000000001101111, OE, 0b1111, 4}},
    {QSA, {"Quick stop active", 0b0000000001101111, QSA, 0b0000, 5}},
    {FRA, {"Fault reaction active", 0b0000000001001111, FRA, 0b0000, 6}},
    {F, {"Fault", 0b0000000001001111, F, 0b0000, 7}},
};

// Print state function
c5e_state parse_state(uint16_t status_word) {
    for (const auto &[key, actuator_state] : states) {
        if ((status_word & actuator_state.mask) == actuator_state.state_id) {
            return actuator_state;
        }
    }
    return states[F];
}

driverless_msgs::msg::Can _d_2_f(uint32_t id, bool is_extended, uint8_t *data, uint8_t dlc) {
    driverless_msgs::msg::Can frame;
    frame.id = id;
    frame.id_type = is_extended;
    std::vector<uint8_t> v;
    v.assign(data, data + dlc);
    frame.data = v;
    frame.dlc = dlc;
    return frame;
}

void sdo_write(uint8_t node_id, uint16_t index, uint8_t cmd, uint8_t *data, size_t data_size, uint32_t *can_packet_id,
               uint8_t *out) {
    *can_packet_id = 0x580 + node_id;
    uint8_t free = 4 - data_size;
    out[0] = cmd;
    if (cmd != 0x60) {                           // not an acknowledge
        out[0] = out[0] | ((free << 2) & 0x1E);  // 4 byte data length (manual pg 119/120)
    }
    out[1] = index & 0xFF;
    out[2] = (index >> 8) & 0xFF;
    out[3] = 0x00;

    for (size_t i = 0; i < data_size; i++) {
        out[i + 4] = data[i];
    }
}
