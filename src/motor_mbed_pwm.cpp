/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor/motor_mbed_pwm.h"

namespace sixtron {

MotorMbedPWM::MotorMbedPWM(float rate_dt,
        MotorSensor *sensor,
        PinName dir,
        PinName pwm,
        PID_params motor_pid,
        float max_pwm):
        _pid(motor_pid, rate_dt), _sensor(sensor), _dir(dir), _pwm(pwm)
{

    _dir.write(0);
    _pwm.period_us(50); // 20kHz
    _pwm.write(0.0f);

    _pid.setLimit(sixtron::PID_limit::output_limit_HL, max_pwm);

    _currentStatus = motor_status::stop;
}

void MotorMbedPWM::init()
{
}

void MotorMbedPWM::start()
{
    if (_currentStatus == motor_status::stop) {
        _currentStatus = motor_status::run;
    }
}

void MotorMbedPWM::stop()
{
    if (_currentStatus == motor_status::run) {
        _currentStatus = motor_status::stop;
    }
}

void MotorMbedPWM::update()
{
    _currentSpeed = _sensor->getSpeed();

    PID_args motor_pid_args;

    motor_pid_args.actual = _currentSpeed;
    motor_pid_args.target = _targetSpeed;

    _pid.compute(&motor_pid_args);

    _motorPwm = motor_pid_args.output;

    // update hardware motor PWM
    if (_motorPwm >= 0.0f) {
        _dir.write(0);
    } else {
        _motorPwm = -_motorPwm;
        _dir.write(1);
    }

    _pwm.write(_motorPwm);
}

void MotorMbedPWM::setSpeed(float speed_ms)
{
    _targetSpeed = speed_ms;
}

float MotorMbedPWM::getSpeed()
{
    return _currentSpeed;
}

} // namespace sixtron
