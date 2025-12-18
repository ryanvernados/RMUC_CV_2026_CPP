// protocol_data.hpp
#pragma once 
#include <cstdint>
#include <cstddef>

struct Protocol {
    enum class Type : std::uint8_t {
        angle_correction = 0x01,
        command = 0x02,
        reserve = 0x03
    };

    struct AngleData {
        float pitch;
        float yaw;
    };

    struct CommandData {
        int fire;
        int aim;
        int chase;
    };

    static constexpr std::uint8_t HEADER_BYTE = 0xAA;
    static constexpr std::size_t MAX_PACKET_SIZE = 256;
    static constexpr std::size_t MAX_PAYLOAD_SIZE = 252;
    static constexpr std::size_t CRC_SIZE = 2;
    static constexpr std::size_t HEADER_SIZE = 4;
};
