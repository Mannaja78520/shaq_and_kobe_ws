from src.utilize import *
import time

class Controller:
    # pidf Setting
    kp = 0
    ki = 0
    kd = 0
    kf = 0
    baseSpeed = 0
    
    Setpoint = 0
    Error = 0
    ErrorTolerance = 0
    LastError = 0
    Integral = 0 
    Dt = 0
    CurrentTime = time.time()
    LastTime = CurrentTime
    
        
    def __init__(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0, errorTolerance = 0):
        self.ConfigPIDF(kp, ki, kd, kf, baseSpeed)
        self.ErrorTolerance = abs(errorTolerance)
        self.LastTime = time.time()
        
    def ConfigPIDF(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.baseSpeed = baseSpeed
    
    def Calculate(self, setpoint, measurement):
        return self.Calculate(setpoint - measurement)
    
    def Calculate(self, error):
        self.CurrentTime = time.time()
        self.Dt = self.CurrentTime - self.LastTime
        self.LastTime  = self.CurrentTime
        Is_Error_In_Tolerance = AtTargetRange(error, 0, self.ErrorTolerance)
        if Is_Error_In_Tolerance :
            return 0
        self.Error = error
        self.Integral += self.Error * self.Dt
        self.Integral = clip(self.Integral, -1, 1)
        Derivative = (self.Error - self.LastError) / self.Dt if self.Dt != 0 else 0
        self.LastError = self.Error
        BaseSpeed = self.baseSpeed * sig_num(self.Error)
        return (self.Error * self.kp) + (self.Integral * self.ki) + (Derivative * self.kd) + (self.Setpoint * self.kf) + BaseSpeed
    
    def ResetVariable(self):
        self.LastError = 0
        self.Integral = 0 
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
        
    def Reset(self, kp = None, ki = None, kd = None, kf = None, baseSpeed = None):
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
