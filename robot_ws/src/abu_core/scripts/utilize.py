#!/usr/bin/env python3
import math

def To_Radians(degs) -> float:
    return math.radians(degs) * -1

def To_Degrees(Rads) -> float:
    return math.degrees(Rads) * -1

def WrapRads(rads) -> float:
    if rads >  math.pi :
        return rads - (2 * math.pi)
    if rads < -math.pi:
        return rads + (2 * math.pi)
    return rads

def WrapDegs(degs) -> float:
    if degs >  180 :
        return degs - 360
    if degs < -180:
        return degs + 360
    return degs

def NormalizeRads(rads):
    rads = rads % (2 * math.pi)
    if rads == (2 * math.pi) :
        return 0
    return rads

def NormalizeDegs(degs):
    degs = degs % 360
    if degs == 360 :
        return 0
    return degs

def sig_num(number) -> int:
    return (number > 0) - (number < 0)

def clip(value, min_val, max_val) -> float:
    return max(min(value, max_val), min_val)

def AtTargetRange(number, target, range) -> bool:
    return abs(number - target) < range