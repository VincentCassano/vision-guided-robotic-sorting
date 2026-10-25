"""Separate immutable identity1000..1005 from diagnostic1006 and voltage1007."""
import copy

STATIC_IDENTITY = [512, 769, 151, 8193, 290, 2]


def identity_fields(raw):
    if (not isinstance(raw, (list, tuple)) or len(raw) != 8
            or any(type(v) is not int or not 0 <= v <= 65535 for v in raw)
            or list(raw[:6]) != STATIC_IDENTITY):
        raise RuntimeError('Actual protocol/firmware/hardware/node identity changed: '+str(raw))
    # Scope only the diagnostic observed in these uploaded results. Do not
    # silently accept calibration, stuck, invalid-data or other sub-errors.
    if raw[6] not in (0, 1) or raw[7] != 0:
        raise RuntimeError('Unsupported hand diagnostic/voltage value: '+str(raw))
    return list(raw[:6])


def same_live_identity(raw, previous):
    identity_fields(raw)
    identity_fields(previous)
    # A diagnostic retained at startup can clear. Once cleared, a new1 is a
    # new event and must terminate normal control, never reopen the token.
    return raw[6] == previous[6] or (previous[6] == 1 and raw[6] == 0)


def configuration_matches(actual, expected):
    if not isinstance(actual, dict) or not isinstance(expected, dict):
        return False
    identity_fields(actual.get('identity'))
    identity_fields(expected.get('identity'))
    # Retain the complete raw values in all reports; compare the same complete
    # calibration/PID/protection configuration and static identity separately.
    a, b = copy.deepcopy(actual), copy.deepcopy(expected)
    a['identity'] = identity_fields(actual['identity'])
    b['identity'] = identity_fields(expected['identity'])
    return a == b
