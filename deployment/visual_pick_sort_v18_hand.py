"""Classify fresh finger posture; this never detects a held object."""


def open_posture(snapshot):
    targets=snapshot.get('targets',[]);positions=snapshot.get('positions',[])
    states=snapshot.get('status',[])
    complete=(len(targets)==len(positions)==len(states)==6
        and all(type(v) is int and 0<=v<=65535 for v in targets+positions)
        and all(type(v) is int and v in (0,1,2,5) for v in states))
    result=dict(method='fresh_five_actual_flex_positions_and_open_target_registers',
        qualified=False,actual_positions=positions[:],target_positions=targets[:],status=states[:],
        five_flex_channels=[0,1,2,3,4],opposition_channel_excluded=True,
        maximum_actual_open_count=150,target_register_zero_tolerance_counts=2,
        tolerance_source='existing_manual_hand_bus_v2_actual_target_arrival_150counts_and_group_readback2counts',
        object_absence_verified=False,physical_empty_hand_sensor_present=False,
        posture_inference_only=True)
    if complete:
        result['qualified']=all(abs(v)<=2 for v in targets[:5]) and all(v<=150 for v in positions[:5])
    return result
