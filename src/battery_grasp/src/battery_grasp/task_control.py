"""Atomic simulation task control; callers serialize command, observe and tick."""
import copy
import math
from .sorting_core import Planner, SimExecutor, build_sequence, validate_objects, require, finite


class TaskController:
    def __init__(self,cfg,now):
        self.cfg = cfg
        self.latest = None
        self.state,self.reason = 'IDLE','Waiting for initial scene'
        self.reset_required = False
        self.mode = 'AUTO'
        self.reset(now)

    def reset(self,now):
        self.planner,self.executor = Planner(self.cfg,require_full_inventory=False),SimExecutor(self.cfg)
        self.latest = None
        self.scene_after = now
        self.scene_ack = None
        self.state,self.reason = 'IDLE','Waiting for fresh scene after reset'
        self.reset_required = False
        self.last_step = now

    def confirm_scene_reset(self,session_id,stamp):
        if session_id==self.planner.session and finite([stamp]) and stamp>=self.scene_after and self.scene_ack is None:
            self.scene_ack = stamp

    def observe(self,snapshot):
        self.latest = copy.deepcopy(snapshot)
        self.executor.observe(snapshot)
        if self.state == 'RUNNING':
            try:
                stamp = snapshot['stamp'] if isinstance(snapshot,dict) else 0
                valid,reason = self.scene_ready(stamp)
                require(valid,reason)
                if self.executor.active is not None:
                    self.executor.fresh(self.executor.active,stamp)
            except (ValueError,KeyError,TypeError,IndexError) as exc:
                self.error(exc)

    def scene_ready(self,now,initial=False):
        try:
            s = self.latest
            require(self.scene_ack is not None,'Waiting for simulated camera reset acknowledgement')
            require(isinstance(s,dict) and type(s.get('schema_version')) is int and s['schema_version']==1,'Invalid perception schema')
            require(s['valid'] is True and s['frame_id']==self.cfg['frame_id'],'Invalid perception frame')
            require(finite([s['stamp'],now]) and self.scene_after < s['stamp'] <= now and now-s['stamp'] <= self.cfg['max_age'],'Waiting for fresh scene')
            require(s['stamp'] >= self.scene_ack,'Waiting for post-reset camera frame')
            validate_objects(s['objects'],self.cfg)
            if initial:
                require(bool(s['objects']),'No remaining battery detected')
                if self.planner.inventory is not None:
                    actual = {o['id']:o for o in s['objects']}
                    require(s['stamp']>self.planner.last_completion_stamp,'Waiting for post-drop frame')
                    require(set(actual)==set(self.planner.inventory)-self.planner.completed,'Waiting for exact remaining inventory; missing or unexpected target')
                    for identity,obj in actual.items():
                        original = self.planner.inventory[identity]
                        require(obj['category']==original['category'] and math.dist(obj['position'],original['position'])<=self.cfg['max_target_shift'],'Recorded target changed')
            return True,''
        except (ValueError,KeyError,TypeError,IndexError) as exc:
            return False,str(exc)

    def status(self,now):
        valid,reason = self.scene_ready(now,initial=True)
        return {'schema_version':1,'state':self.state,'reason':self.reason,'stamp':now,
                'ready':self.state=='IDLE' and not self.reset_required and valid,
                'readiness_reason':reason,'reset_required':self.reset_required,
                'mode':self.mode,'session_id':self.planner.session,'completed':sorted(self.planner.completed),
                'total':len(self.planner.inventory) if self.planner.inventory is not None else None,
                'remaining':len(self.planner.inventory)-len(self.planner.completed) if self.planner.inventory is not None else None,
                'current_target':self.planner.current['target']['id'] if self.planner.current else None,
                'simulation_only':True}

    def command(self,command,now):
        code = 200
        reset_event = None
        if command in ('MODE_MANUAL','MODE_AUTO'):
            if self.state not in ('IDLE','DONE'):
                code = 409
            else:
                self.mode = command.split('_')[1]
        elif command=='START':
            if self.state=='RUNNING':
                code = 409 if self.mode=='MANUAL' else 200
            elif self.state!='IDLE' or self.reset_required:
                code = 409
            else:
                valid,reason = self.scene_ready(now,initial=True)
                if not valid:
                    code,self.reason = 409,reason
                else:
                    if self.planner.inventory is None:
                        self.planner.inventory = {obj['id']:copy.deepcopy(obj) for obj in self.latest['objects']}
                    self.state,self.reason = 'RUNNING','START accepted'
                    self.executor.set_running(True)
                    self.last_step = now
        elif command=='STOP':
            if self.state=='RUNNING':
                self.executor.set_running(False)
                self.state,self.reason = 'STOPPING','No further steps permitted; RESET required'
                self.reset_required = True
                code = 202
            elif self.state=='STOPPING':
                code = 202
        elif command=='RESET':
            if self.state in ('RUNNING','STOPPING'):
                code = 409
            else:
                self.reset(now)
                reset_event = {'schema_version':1,'session_id':self.planner.session,'stamp':now}
        else:
            code = 400
        response = self.status(now)
        response.update(http_status=code,accepted=code<400,command=command)
        if code==409:
            response['error'] = 'Command not allowed in current state or scene not ready'
        elif code==400:
            response['error'] = 'Unknown command'
        return response,reset_event

    def error(self,reason):
        self.executor.set_running(False)
        self.state,self.reason = 'ERROR',str(reason)
        self.reset_required = True

    def tick(self,now):
        output = {'task':None,'sequence':None,'event':None,'result':None}
        if self.state=='STOPPING':
            self.state,self.reason = 'IDLE','Stopped; RESET required before START'
            return output
        if self.state!='RUNNING':
            return output
        try:
            valid,reason = self.scene_ready(now)
            require(valid,reason)
            if self.planner.current is None:
                task = self.planner.observe(self.latest['objects'],self.latest['stamp'],now)
                if self.planner.status=='DONE':
                    self.state,self.reason = 'DONE','All recorded batteries sorted'
                    self.reset_required = True
                    self.executor.set_running(False)
                    return output
                if task is None:
                    # A dropped battery may remain in a queued camera frame briefly.
                    require(now-self.planner.last_completion_stamp <= self.cfg['max_age'],self.planner.reason)
                    return output
                sequence = build_sequence(task,self.cfg)
                require(self.executor.accept(sequence,now),'Executor refused task')
                output['task'],output['sequence'] = task,sequence
                self.last_step = now
                self.reason = 'Executing '+task['target']['id']
            if now-self.last_step >= self.cfg['step_delay']:
                self.last_step = now
                event,result = self.executor.tick(now)
                output['event'],output['result'] = event,result
                if result:
                    if result['status']!='DONE':
                        self.error(result.get('reason','Execution failed'))
                    else:
                        require(self.planner.finish(result),'Completion rejected')
                        if self.mode=='MANUAL':
                            self.executor.set_running(False)
                            self.state,self.reason = 'IDLE','One battery sorted; waiting for next command'
        except (ValueError,KeyError,TypeError,IndexError) as exc:
            self.error(exc)
        return output
