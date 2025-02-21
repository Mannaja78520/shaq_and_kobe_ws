#!/usr/bin/env python3

from src.utilize import *
import time

class Controller:
    # pidf Setting
    kp: float = 0
    ki: float = 0
    kd: float = 0
    kf: float = 0
    baseSpeed: float = 0
    
    Setpoint: float = 0
    Error: float = 0
    ErrorTolerance: float = 0
    LastError: float = 0
    Integral: float = 0 
    i_min: float = 0
    i_max: float = 0
    Dt = 0
    CurrentTime = time.time()
    LastTime = CurrentTime
    
        
    def __init__(self, kp: float = 0, ki: float = 0, kd: float = 0, kf: float = 0, baseSpeed: float = 0, errorTolerance: float = 0, i_min: float = 0, i_max: float = 0):
        self.ConfigPIDF(kp, ki, kd, kf, baseSpeed, i_min, i_max)
        self.ErrorTolerance = abs(errorTolerance)
        self.LastTime = time.time()
        
    def ConfigPIDF(self, kp: float = 0, ki: float = 0, kd: float = 0, kf: float = 0, baseSpeed: float = 0, i_min: float = 0, i_max: float = 0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.baseSpeed = baseSpeed
        if i_min == 0 and i_max != 0:
            self.i_min = -i_max
            self.i_max = i_max
            return
        self.i_min = i_min
        self.i_max = i_max
    
    def CalculateWithSetpoint(self, setpoint: float, measurement: float) -> float:
        self.Setpoint = setpoint
        return self.Calculate(setpoint - measurement)

    def Calculate(self, error : float) -> float:
        self.CurrentTime = time.time()
        self.Dt = self.CurrentTime - self.LastTime
        self.LastTime  = self.CurrentTime
        Is_Error_In_Tolerance = AtTargetRange(error, 0, self.ErrorTolerance)
        if Is_Error_In_Tolerance :
            return (self.Setpoint * self.kf)
        # if (self.Setpoint == 0 and self.kf > 0.0) :
        #     return 0 
        self.Error = error
        self.Integral += self.Error * self.Dt
        self.Integral = clip(self.Integral, self.i_min, self.i_max)
        Derivative = (self.Error - self.LastError) / self.Dt if self.Dt != 0 else 0
        self.LastError = self.Error
        BaseSpeed = self.baseSpeed * sig_num(self.Error)
        return (self.Error * self.kp) + (self.Integral * self.ki) + (Derivative * self.kd) + (self.Setpoint * self.kf) + BaseSpeed

    def ResetVariable(self) -> None:
        self.LastError = 0
        self.Integral = 0
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
        
    def Reset(self, kp = None, ki = None, kd = None, kf = None, baseSpeed = None) -> None:
        # pidf Setting
        kp = self.kp if kp == None else kp
        ki = self.ki if ki == None else ki
        kd = self.kd if kd == None else kd
        kf = self.kf if kf == None else kf
        baseSpeed = self.baseSpeed if baseSpeed == None else baseSpeed
        self.ConfigPIDF(kp, ki, kd, kf, baseSpeed)
            
        # pid variable
        self.Setpoint = 0
        self.Error = 0
        self.LastError = 0
        self.Integral = 0 
        self.Dt = 0
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
