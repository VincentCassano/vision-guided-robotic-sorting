"""School battery sequence adapter. Offline by default; no hardware imports."""
import argparse
import hashlib
import json
from pathlib import Path

CHANNELS=('thumb_flex','index','middle','ring','little','thumb_rotation')
# Demonstrated targets, NOT certified mechanical or collision-safe limits.
OBSERVED_MAX=(0,7500,7500,7500,5000,10000)

def compile_plan(recipe):
    if recipe.get('schema_version')!=1 or recipe.get('channel_order')!=list(CHANNELS):
        raise ValueError('Unsupported recipe schema or channel mapping')
    result=[]
    for phase in ('prepare','grasp'):
        for source in recipe[phase]:
            ch=source['start_channel'];values=source['values'];count=source['count']
            if type(ch) is not int or not 0<=ch<6 or count!=len(values) or ch+count>6:
                raise ValueError('Invalid channel range')
            if source['address']!=1135+ch or any(type(v) is not int or not 0<=v<=65535 for v in values):
                raise ValueError('Invalid target or register')
            payload=[b for v in values for b in (v>>8,v&255)]
            if payload!=source['bytes_high_low']:raise ValueError('Byte order mismatch')
            result.append(dict(phase=phase,address=1135+ch,count=count,values=values[:],
                bytes_high_low=payload,source_delay_seconds=source['delay_seconds_source_default']))
    return result

def review(plan):
    gaps=[];pose=[None]*6
    for index,step in enumerate(plan):
        for offset,value in enumerate(step['values']):
            ch=step['address']-1135+offset;pose[ch]=value
            if value>OBSERVED_MAX[ch]:
                gaps.append(dict(step=index,channel=CHANNELS[ch],requested=value,observed_max=OBSERVED_MAX[ch]))
    return dict(execution_ready=False,final_pose=pose,unverified_targets=gaps,
        unresolved=['School inherited speed is unspecified.',
                    'Contact stopping and release are not verified.',
                    'Prior status 5 cause is unresolved.'])

def replay_offline(plan):
    """Register replay only. Never imports school modules or sends commands."""
    registers={};rows=[]
    for i,step in enumerate(plan):
        for j,value in enumerate(step['values']):registers[step['address']+j]=value
        rows.append(dict(step=i,phase=step['phase'],pose=[registers.get(1135+c) for c in range(6)]))
    return rows

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('captures/school_battery_plan.json'))
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    source=Path(__file__).with_name('school_battery_pose.json')
    recipe=json.loads(source.read_text(encoding='utf-8'));plan=compile_plan(recipe)
    result=dict(recipe_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),plan=plan,
                review=review(plan),replay=replay_offline(plan),hardware_connected=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    tmp=args.output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');tmp.replace(args.output)
    print('SAVED:',args.output)
    print('School final pose:',result['review']['final_pose'])
    if args.execute:
        print('BLOCKED before connection: unverified full-range targets and stop/release. No writes.')
        return 2
    print('OFFLINE ONLY. No robot connection or motion.')
    return 0

if __name__=='__main__':raise SystemExit(main())
