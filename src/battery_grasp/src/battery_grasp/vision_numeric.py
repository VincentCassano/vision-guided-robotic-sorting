"""Numeric representation allowance, separate from physical vision limits."""
import math

DEPTH_SPREAD_LIMIT_M=.005
FLOAT32_RELATIVE_EPSILON=2.0**-23


def depth_spread_check(value):
    """The inclusive 5mm limit allows one float32 relative rounding unit.

    RealSense depth_scale is returned as float32 and the recorded percentile
    spread is calculated in double precision. Five 1mm units can consequently
    be serialized as 0.0050000002374872565m. This sub-nanometre arithmetic
    allowance is not a sensor error budget or a new physical noise threshold.
    """
    if type(value) not in (int,float) or not math.isfinite(value):
        raise ValueError('Depth spread must be a finite number')
    allowance=DEPTH_SPREAD_LIMIT_M*FLOAT32_RELATIVE_EPSILON
    return dict({'pass':0<=value<=DEPTH_SPREAD_LIMIT_M+allowance},
        observed_m=value,configured_inclusive_limit_m=DEPTH_SPREAD_LIMIT_M,
        float32_relative_epsilon=FLOAT32_RELATIVE_EPSILON,
        numerical_roundoff_allowance_m=allowance,
        comparison_upper_m=DEPTH_SPREAD_LIMIT_M+allowance,
        allowance_kind='float32_representation_only_not_sensor_accuracy_or_physical_limit_change')
