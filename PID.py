#!/usr/bin/env python
from time import time, sleep

class PIDcontroller:
    def __init__(self, Ki=0, Kp = 0, Kd =0):
        #gains#
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd

        #memory#
        self.prev_error = 0.0
        self.I = 0.0
        self.D = 0.0
        self.prev_back_signal = 0.0
        self.T_start = time()
        self.output = 0.0

        #other parameters#
        self.tau = 0.02
        self.limMin = -10
        self.limMax = 10
        self.limMinInt = -5.0
        self.limMaxInt = 5.0

    def T(self):
        return round(time() - self.T_start, 2)

    def updatePID(self, input_signal, back_signal): #функция для вычисления ПИД-регулирования
        error = input_signal - back_signal

        P = self.Kp * error

        self.I = self.I + 0.5*self.Ki*self.T()*(error + self.prev_error)
        if self.I > self.limMaxInt:
            self.I = self.limMaxInt
        elif self.I < self.limMinInt:
            self.I = self.limMinInt

        self.D = - (2.0 * self.Kd * (back_signal - self.prev_back_signal) + (2.0*self.tau - self.T()) * self.D) / (2.0*self.tau + self.T())

        self.output = P + self.I + self.D
        if self.output > self.limMax:
            self.output = self.limMax
        elif self.output < self.limMin:
            self.output = self.limMin

        self.prev_error = error
        self.prev_back_signal = back_signal

        return self.output
    
    #     I = self.Ki * self.errorSum
    #     D = self.Kd * self.derrdt()
    #     print(f'P: {P}')
    #     print(f'I: {I}')
    #     print(f'D: {D}')
    #     return P+I+D
    
    # def derrdt(self, dt=2): # dt - окно дискретизации
    #     if (self.prev_error is not None) and (self.cur_error is not None):
    #         return (self.cur_error - self.prev_error)/dt
    #     else:
    #         return 0
    
    # def calculate(self, input_sig, back_sig): #вычисление выходного сигнала после ПИД-регулирования
    #     error_sig = input_sig - back_sig
    #     self.errorSum += error_sigclock
    #         self.cur_error = error_sig
    #     else:
    #         self.prev_error = self.cur_error
    #         self.cur_error = error_sig
    #     return self.updatePID()

def test_func(inp):
    output = 0.0
    alpha = 0.02
    output = (0.01*inp+output)/(1.0+alpha*0.01)
    return output

if __name__ == "__main__":
    pid = PIDcontroller(Ki = 0, Kp=1.010099, Kd = 0)
    for i in range(50):
        back_signal = test_func(pid.output)
        print(f'time: {pid.T()}\npid :{pid.updatePID(1, back_signal)}')
        sleep(0.1)

