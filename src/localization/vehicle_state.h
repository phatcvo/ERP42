#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "point.h"
#include <iostream>

class VehicleState {
public:
    VehicleState(float x = 0, float y = 0, float yaw = 0, float velocity = 0)
        : position_(x, y), yaw_(yaw), velocity_(velocity) {}

    Point position() const { return position_; }
    float yaw() const { return yaw_; }
    float velocity() const { return velocity_; }

    std::string str() const {
        return "VehicleState(position: (" + std::to_string(position_.x()) + ", " + std::to_string(position_.y()) +
               "), yaw: " + std::to_string(yaw_) + ", velocity: " + std::to_string(velocity_) + ")";
    }

private:
    Point position_;
    float yaw_;
    float velocity_;
};

#endif // VEHICLE_STATE_H
