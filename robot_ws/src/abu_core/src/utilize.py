import math

def To_Radians(degs):
    return math.radians(degs) * -1

def To_Degrees(Rads):
    return math.degrees(Rads) * -1

def WrapRads(rads):
    if rads >  math.pi :
        return rads - (2 * math.pi)
    if rads < -math.pi:
        return rads + (2 * math.pi)
    return rads

def WrapDegs(degs):
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

def sig_num(number):
    return -1 if number < 0 else 1 if number > 0 else 0

def clip(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def AtTargetRange(number, target, range):
    return abs(number - target) < range
