#!/usr/bin/env python3


class PID():
    def __init__(self, param):
        self.param = param
        self.integrator = 0
        self.derivator = 0

    def sign(self, value):
        '''return +/-1 for positive negative input'''
        return 0 if value == 0 else value/abs(value)

    def integrator_value(self, error):
        self.integrator = error + self.integrator
        if abs(self.integrator) > self.param.integrator_max:
            self.integrator = self.param.integrator_max*self.sign(self.integrator)
        return self.integrator

    def derivator_value(self, error):
        derivator_value = error - self.derivator
        self.derivator = error
        return derivator_value

    def limit_signal(self, signal):
        '''limit PWM signal to meaningfull values'''
        if signal == 0 or abs(signal)<self.param.limit_min:
            signal = 0
        elif abs(signal) > self.param.limit_max:
            signal = self.param.limit_max*self.sign(signal)
        else:
            signal = signal
        return signal

    def pid(self, signal_in, feedback):
        error = signal_in - feedback
        p = self.param.kp*error
        i = self.param.ki*self.integrator_value(error)
        d = self.param.kd*self.derivator_value(error)
        return self.limit_signal(p + i + d)