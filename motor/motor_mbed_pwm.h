/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTOR_MBED_PWM_H
#define CATIE_SIXTRON_MOTOR_MBED_PWM_H

#include "mbed.h"
#include "motor/motor.h"
#include "motor_sensor/motor_sensor.h"
#include "pid/pid.h"

namespace sixtron {

#define DEFAULT_MOTOR_MAX_PWM 1.0f // max PWM with mbed is 1.0f

class MotorMbedPWM: Motor {

public:
    MotorMbedPWM(float rateHz,
            MotorSensor *sensor,
            PinName dir,
            PinName pwm,
            PID_params motor_pid,
            float max_pwm = DEFAULT_MOTOR_MAX_PWM);

    void init() override;

    void start() override;

    void stop() override;

    void update() override;

    void setSpeed(float speed_ms) override;

    float getSpeed() override;

private:
    PID _pid;
    MotorSensor *_sensor;
    DigitalOut _dir;
    PwmOut _pwm;

    motor_status _currentStatus;
    float _targetSpeed = 0.0f;
    float _currentSpeed = 0.0f;
    float _motorPwm = 0.0f;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_MBED_PWM_H
