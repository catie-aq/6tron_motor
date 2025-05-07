/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTOR_DC_H
#define CATIE_SIXTRON_MOTOR_DC_H

#include "motor/motor.h"
#include "pid/pid.h"

namespace sixtron {

#define DEFAULT_MOTOR_MAX_PWM 1.0f

class MotorDC: Motor {

public:
    MotorDC(float rate_dt, PID_params motor_pid, float max_pwm = DEFAULT_MOTOR_MAX_PWM);

    void init() override;

    void start() override;

    void stop() override;

    void standby() override;

    void update() override;

    void setSpeed(float speed_ms) override;

    float getSpeed() override;

protected:
    virtual void initHardware() = 0;
    virtual void setPWM(float pwm) = 0; // should be between -1.0f and +1.0f !
    virtual float getSensorSpeed() = 0; // Should be in m/s !

private:
    PID _pid;
    motor_status _currentStatus;
    float _targetSpeed = 0.0f;
    float _currentSpeed = 0.0f;
    float _motorPwm = 0.0f;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_DC_H
