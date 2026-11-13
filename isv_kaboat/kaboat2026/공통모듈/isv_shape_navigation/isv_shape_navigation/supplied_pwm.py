"""User's 2026-11-04 PWM tables with an optional forward-base increase.

Angles here are clockwise/right-positive degrees. ROS body yaw is CCW;
the caller explicitly negates its bearing before invoking these functions.
"""
import math


def checked(distance, angle):
    if not math.isfinite(distance) or distance < 0 or not math.isfinite(angle):
        raise ValueError('Finite nonnegative distance and finite angle required')


def clamp_pair(left, right):
    return max(1100, min(1900, int(left))), max(1100, min(1900, int(right)))


def heading_pwm(distance, angle, holding=False, forward_boost=0):
    checked(distance, angle)
    if holding:
        base = 1500
        if abs(angle) <= 1:
            return base, base
        if 1 < angle <= 3:
            diff = 15
        elif -3 <= angle < -1:
            diff = -25
        elif abs(angle) <= 8:
            diff = 35 if angle > 0 else -35
        elif abs(angle) <= 15:
            diff = 50 if angle > 0 else -50
        else:
            diff = 60 if angle > 0 else -60
        return clamp_pair(base+diff, base-diff)
    base = 1600+forward_boost
    if distance < 5:
        base = int(1500+100*max(.7, distance/5.))+forward_boost
    if abs(angle) <= 1.5:
        return base, base
    for limit, delta in ((4,25), (8,45), (15,70), (25,100), (40,140), (60,190)):
        if abs(angle) <= limit:
            diff = delta if angle > 0 else -delta
            return clamp_pair(base+diff, base-diff)
    if abs(angle) <= 90:
        return (1800, 1300) if angle > 0 else (1300, 1800)
    return (1850, 1200) if angle > 0 else (1200, 1850)


def obstacle_pwm(distance, angle, forward_boost=0):
    checked(distance, angle)
    base = 1680+forward_boost
    multiplier = .75+min(1., distance/1.5)*.25
    base = 1500 + (base - 1500) * multiplier
    angle = max(-75., min(75., angle))
    if abs(angle) <= 2:
        return clamp_pair(base, base)
    for limit, amount in ((8,60), (15,110), (30,150), (50,185), (75,210)):
        if abs(angle) <= limit:
            diff = int(amount*multiplier)*(1 if angle > 0 else -1)
            return clamp_pair(base+diff, base-diff)
    raise AssertionError('Clamped angle must match PWM table')


def soften_turn_pwm(pwm, scale=.5, max_difference=200):
    """Reduce differential thrust, preserving pair mean and exact neutral."""
    left,right=pwm
    if any(type(v) is not int or not 1100<=v<=1900 for v in pwm):
        raise ValueError('Invalid source PWM')
    if not math.isfinite(scale) or not 0<scale<=1:
        raise ValueError('Invalid turning scale')
    if type(max_difference) is not int or not 1<=max_difference<=800:
        raise ValueError('Invalid turning difference limit')
    mean=(left+right)/2.
    difference=max(-max_difference,min(max_difference,(left-right)*scale))
    return clamp_pair(round(mean+difference/2.),round(mean-difference/2.))


def amplify_turn_pwm(pwm, gain):
    """Increase left/right differential while preserving mean thrust."""
    left, right = pwm
    if any(type(v) is not int or not 1100 <= v <= 1900 for v in pwm):
        raise ValueError('Invalid source PWM')
    if not math.isfinite(gain) or not 1 <= gain <= 2:
        raise ValueError('Invalid turning gain')
    mean = (left + right) / 2.
    difference = (left - right) * gain
    return clamp_pair(round(mean + difference / 2.), round(mean - difference / 2.))
