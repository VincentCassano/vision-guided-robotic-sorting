#!/usr/bin/env python3
"""Feedback policy for supervised, previously accepted EMPTY-hand poses.

Status 5 is recorded, not treated as a stand-alone abort or as a proven firmware
bug.  Actual position feedback determines settling.  Sequential register reads
cannot certify physical stopping, contact detection or torque release.
"""
from run_school_battery_hand import Hand, IDENTITY, ProtectionFault, TOL, TARGET_TOL


class FeedbackHand(Hand):
    """Keep device protection and command boundaries; tolerate status-only 5.

    This class never retransmits a target and never writes protection settings.
    Its callers must restrict poses to the accepted empty-hand profile and keep
    an onsite observer ready to operate hardware stop.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.report['feedback_policy'] = dict(
            name='actual_position_settling_status5_warning',
            status5_firmware_bug_confirmed=False,
            status5_alone_aborts=False,
            position_tolerance=TOL,
            target_readback_tolerance=TARGET_TOL,
            position_window_criterion='all_sampled_positions_within_target_tolerance',
            adjacent_position_gate=False,
            observed_settling_seconds=5,
            reads_are_sequential=True,
            physical_stop_verified=False,
            torque_release_verified=False,
            empty_hand_only=True)

    def _protection(self, message):
        self.report['protection_latched'] = True
        self.io.disable_motion()
        self.save()
        raise ProtectionFault(message)

    @staticmethod
    def _u16(values, length, name):
        if len(values) != length or any(type(v) is not int or not 0 <= v <= 65535
                                        for v in values):
            raise RuntimeError('Invalid '+name+' feedback')

    def snapshot(self):
        started = self.clock()
        status = self.read('status', 1085, 6)
        if len(status) != 6 or any(type(v) is not int or v not in (0, 1, 2, 3, 4, 5)
                                    for v in status):
            self._protection('Invalid status feedback: '+str(status))
        # These distinct protection/control-mode indications stay blocking.
        if any(v in (3, 4) for v in status):
            self._protection('Current protection or unexpected force-control status: '+str(status))
        current = self.read('current_ma_0_to_4', 1105, 5)
        rotation_current = self.io.single(1110)
        self.event('read', name='rotation_current_ma', address=1110,
                   values=[rotation_current])
        positions = self.read('positions', 1145, 6)
        currents = current + [rotation_current]
        self._u16(currents, 6, 'current')
        self._u16(positions, 6, 'position')
        limits = self.report['configuration']['current_limit_ma']
        self._u16(limits, 6, 'current limit')
        snap = dict(status=status, current=currents, positions=positions,
                    reads_are_sequential=True, started_monotonic=started,
                    finished_monotonic=self.clock())
        self.report['last_snapshot'] = snap
        if any(limit <= 0 or c > limit for c, limit in zip(currents, limits)):
            self._protection('Measured current exceeds live controller current limit')
        if 5 in status:
            self.report['status5_warning_count'] = self.report.get('status5_warning_count', 0) + 1
            self.event('status5_warning', snapshot=snap,
                       message='Status 5 alone is not a completion or abort decision; reads are sequential',
                       no_retry_or_target_write=True)
        self.save()
        return snap

    def preflight(self, expected=None, start_open=False):
        identity = self.read('identity', 1000, 8)
        self._u16(identity, 8, 'identity')
        if expected is not None:
            self._u16(expected, 6, 'expected target')
        if identity[:6] != IDENTITY or identity[6] != 0:
            self.io.disable_motion()
            raise RuntimeError('Unexpected identity or nonzero sub-exception: '+str(identity))
        snap = self.snapshot()
        targets = self.read('targets', 1135, 6)
        speeds = self.read('speeds', 1125, 6)
        forces = self.read('force_targets', 1115, 5)
        for values, length, name in ((targets, 6, 'target'), (speeds, 6, 'speed'),
                                      (forces, 5, 'force target')):
            self._u16(values, length, name)
        values = dict(identity=identity, **snap, targets=targets,
                      speeds=speeds, force_targets=forces)
        self.report['last_preflight'] = values
        self.save()
        if any(forces):
            raise RuntimeError('Require zero force-control targets')
        if any(abs(a-b) > TOL for a, b in zip(targets, snap['positions'])):
            raise RuntimeError('Targets do not match positions within feedback tolerance')
        if expected is not None:
            if any(abs(a-b) > TARGET_TOL for a, b in zip(targets, expected)):
                raise RuntimeError('Target changed outside this script')
            if speeds != self.report.get('required_speeds', [65535]*6):
                raise RuntimeError('Recorded speed profile changed; no next pose')
        if start_open and (any(v > TOL for v in targets[:5]+snap['positions'][:5])
                           or not 0 <= targets[5] <= 2000
                           or not 0 <= snap['positions'][5] <= 2150):
            raise RuntimeError('Start requires open flex fingers and thumb at side-palm')
        return values

    def wait(self, expected, active_channels, previous, min_seconds=1.0):
        self._u16(expected, 6, 'expected target')
        self._u16(previous, 6, 'previous target')
        if any(type(c) is not int or not 0 <= c <= 5 for c in active_channels):
            raise RuntimeError('Invalid active channel')
        begin = self.clock()
        speeds = self.report.get('required_speeds', [65535]*6)
        self._u16(speeds, 6, 'speed')
        if any(speeds[c] == 0 for c in active_channels):
            raise RuntimeError('Zero active-channel speed; no settling budget')
        travel = max((abs(expected[c]-previous[c])/speeds[c]
                      for c in active_channels), default=0)
        deadline = begin+travel+30
        settled_since = None
        stable = 0
        window_minimum = window_maximum = None
        while self.clock() < deadline:
            snap = self.snapshot()
            positions, status = snap['positions'], snap['status']
            targets = self.read('targets_during_motion', 1135, 6)
            self._u16(targets, 6, 'target')
            if any(abs(a-b) > TARGET_TOL for a, b in zip(targets, expected)):
                self.event('target_changed_during_motion', expected=expected, actual=targets)
                raise RuntimeError('Target changed outside this script: '+str(targets))
            for ch in range(6):
                if ch not in active_channels and abs(positions[ch]-previous[ch]) > TOL:
                    self.event('inactive_position_drift', channel=ch,
                               expected=previous[ch], actual=positions[ch],
                               status=status[ch], targets=targets)
                    raise RuntimeError('Inactive finger position drift exceeds tolerance: '+str(ch))
                if not max(0, min(previous[ch], expected[ch])-TOL) <= positions[ch] <= min(
                        65535, max(previous[ch], expected[ch])+TOL):
                    raise RuntimeError('Position outside commanded travel: '+str(positions))
            observed = self.clock()
            reached = all(abs(a-b) <= TOL for a, b in zip(positions, expected))
            if reached:
                # TARGET_TOL=2 is only command readback quantization. The real
                # 202513 feedback varied14970..15020 at target15000: resetting
                # the observation on this normal in-tolerance jitter incorrectly
                # timed out. Require every sampled position within TOL instead.
                if settled_since is None:
                    settled_since, stable = observed, 1
                    window_minimum, window_maximum = positions[:], positions[:]
                else:
                    stable += 1
                    window_minimum = [min(a,b) for a,b in zip(window_minimum,positions)]
                    window_maximum = [max(a,b) for a,b in zip(window_maximum,positions)]
            else:
                settled_since, stable = None, 0
                window_minimum = window_maximum = None
            if (observed <= deadline and reached and stable >= 2
                    and observed-settled_since >= 5
                    and observed-begin >= min_seconds):
                self.event('actual_position_reached_and_observed', expected=expected,
                           positions=positions, status=status,
                           elapsed_seconds=observed-begin,
                           settling_seconds=observed-settled_since,
                           stable_samples=stable,
                           position_window_minimum=window_minimum,
                           position_window_maximum=window_maximum,
                           position_window_span=[b-a for a,b in zip(window_minimum,window_maximum)],
                           reads_are_sequential=True,
                           physical_stop_verified=False,
                           torque_release_verified=False)
                return snap
        raise RuntimeError('Pose timeout; no automatic return')
