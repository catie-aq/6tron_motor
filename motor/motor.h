/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTOR_H
#define CATIE_SIXTRON_MOTOR_H

#include <math.h>
#include <stdint.h>

namespace sixtron {

typedef enum {
    run,
    stop
} motor_status;

class Motor {

public:
    virtual void init() = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual void update() = 0; // Update Motor Output. Useful when pid is done in soft by user and
                               // not by motor hardware.

    virtual void setSpeed(float speed_ms) = 0;

    virtual float getSpeed() = 0;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_H
