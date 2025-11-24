#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <string>

struct SensorData {
    std::string up;
    std::string down;
    std::string left;
    std::string right;
    std::string up_left;
    std::string up_right;
    std::string down_left;
    std::string down_right;
    
    // Helper para verificar se há parede
    bool isWall(const std::string& sensor) const {
        return sensor == "b";
    }
};

#endif
