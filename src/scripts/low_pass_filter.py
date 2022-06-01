#!/usr/bin/env python

from scipy import signal
from math import pi 


class filter:
    def __init__(self, w):
        self.w = w 
        self.samplingFreq = 1000
        self.num = 2*pi*w
        self.den = [1, self.num]
        self.a = [] 
        self.b = []

    def filterCofficient(self):
        lowPass = signal.TransferFunction(self.num,self.den)
        dt = 1.0/self.samplingFreq
        discreteLowPass = lowPass.to_discrete(dt,method='gbt',alpha=0.5)
        return -discreteLowPass.den[1:],discreteLowPass.num
