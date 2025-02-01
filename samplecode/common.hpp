#pragma once
#include <cstdint>
#include <vector>
#include <string>

struct SensorData {
    std::vector<float> temperatures;
    std::vector<int16_t> positions;
    float batteryLevel;
    int16_t status;
};