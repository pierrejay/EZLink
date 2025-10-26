/* @file Prototypes.h */

#pragma once
#include <EZLink.hpp>

using MsgType = ezlink::MsgType;

struct SetLedMsg {
    static constexpr MsgType type = ezlink::MESSAGE;
    static constexpr uint8_t id = 1;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} __attribute__((packed));

struct SensorDataResp {
    static constexpr MsgType type = ezlink::RESPONSE;
    static constexpr uint8_t id = 2;
    float temperature;
    float humidity;
} __attribute__((packed));

struct GetSensorDataReq {
    static constexpr MsgType type = ezlink::REQUEST;
    static constexpr uint8_t id = 2;
    uint8_t sensor_id;
    using ResponseType = SensorDataResp;
} __attribute__((packed));
