/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor/motor_DC.h"

namespace sixtron {

MotorDC::MotorDC(float rate_dt, PID_params motor_pid, float max_pwm): _pid(motor_pid, rate_dt)
{
    _pid.setLimit(sixtron::PID_limit::output_limit_HL, max_pwm);

    _currentStatus = motor_status::stop;
}

void MotorDC::init()
{
    initHardware();
}

void MotorDC::start()
{
    if (_currentStatus == motor_status::stop) {
        _currentStatus = motor_status::run;
    }
}

void MotorDC::stop()
{
    if (_currentStatus == motor_status::run) {
        _currentStatus = motor_status::stop;
    }
}

void MotorDC::standby()
{
    _pid.reset();
    _motorPwm = 0.0f;
    setPWM(_motorPwm);
}

void MotorDC::update()
{
    _currentSpeed = getSensorSpeed();

    PID_args motor_pid_args;

    motor_pid_args.actual = _currentSpeed;
    motor_pid_args.target = _targetSpeed;

    _pid.compute(&motor_pid_args);

    _motorPwm = motor_pid_args.output;

    setPWM(_motorPwm);
}

void MotorDC::setSpeed(float speed_ms)
{
    _targetSpeed = speed_ms;
}

float MotorDC::getSpeed()
{
    return _currentSpeed;
}

} // namespace sixtron
