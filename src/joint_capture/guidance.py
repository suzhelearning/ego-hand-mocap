"""Non-blocking calibration prompts; timestamps describe prompts, not measured motion."""
import math
from .storage import save_json


def build_plan(target):
    if target not in ('rbc','tbc'):
        raise ValueError('未知标定目标')
    steps=[]
    def add(instruction,duration,kind,**extra):
        steps.append(dict(id=f'step_{len(steps)+1:03d}',instruction=instruction,
                          duration_s=duration,kind=kind,**extra))
    add('保持初始姿态静止，目视前方纹理丰富的场景',5,'hold')
    locations=[('center','中心位置')]
    if target=='tbc':locations += [('left_front','左前方约20～30 cm'),('right_front','右前方约20～30 cm')]
    axes=[('yaw','向左转约40°','向右转约40°'),('pitch','抬头约30°','低头约30°'),('roll','向左侧倾约25°','向右侧倾约25°')]
    for location,label in locations:
        if location!='center':add(f'缓慢移动到{label}，恢复正向姿态',5,'relocate',location=location)
        for axis,left,right in axes:
            for repetition in range(1,(3 if target=='rbc' else 2)+1):
                extra=dict(axis=axis,location=location,repetition=repetition)
                for direction in (left,right):
                    add(f'{label}：缓慢{direction}',2,'move',**extra)
                    add(f'{label}：保持{direction}的姿态静止',2,'hold',**extra)
                    add(f'{label}：缓慢回正',2,'move',**extra)
    add('保持结束姿态静止，等待自动停止录制',5,'hold')
    return steps


class CalibrationGuide:
    def __init__(self,target,path,event,output=None,*,plan=None):
        self.path=path;self.event=event
        self.output=output or (lambda text:print(text,flush=True))
        self.plan=build_plan(target) if plan is None else plan
        self.state=dict(schema='ego-calibration-guide-v1',target=target,status='pending',
                        time_domain='linux-clock-monotonic',
                        timestamp_semantics='Instruction boundaries, not detected physical motion or camera exposure',
                        planned_duration_s=sum(s['duration_s'] for s in self.plan),
                        plan=self.plan,segments=[])
        self.index=0;self.last_display=None
        self.save()

    def save(self):save_json(self.path,self.state)

    def start(self,now):
        self.state.update(status='running',start_pc_monotonic_ns=now)
        self._enter(now)

    def _enter(self,now):
        step=self.plan[self.index]
        self.state['segments'].append(dict(step_id=step['id'],start_pc_monotonic_ns=now,status='running'))
        self.event('calibration_step_start',step_id=step['id'],target=self.state['target'],boundary_pc_monotonic_ns=now,instruction=step['instruction'])
        self.save();self.last_display=None;self._display(now)

    def _display(self,now):
        step=self.plan[self.index];start=self.state['segments'][-1]['start_pc_monotonic_ns']
        remaining=max(0,math.ceil(step['duration_s']-(now-start)/1e9))
        if remaining!=self.last_display:
            self.output(f"[{self.state['target'].upper()} {self.index+1}/{len(self.plan)}] {step['instruction']} ｜剩余 {remaining} 秒")
            self.last_display=remaining

    def _end(self,now,status):
        segment=self.state['segments'][-1]
        segment.update(end_pc_monotonic_ns=now,status=status)
        self.event('calibration_step_end',step_id=segment['step_id'],status=status,boundary_pc_monotonic_ns=now)

    def tick(self,now):
        if self.state['status']=='complete':return True
        segment=self.state['segments'][-1]
        if (now-segment['start_pc_monotonic_ns'])/1e9>=self.plan[self.index]['duration_s']:
            self._end(now,'complete');self.index+=1
            if self.index==len(self.plan):
                self.state.update(status='complete',end_pc_monotonic_ns=now)
                self.save();self.output('标定动作已完成，自动停止并保存。');return True
            self._enter(now)
        else:self._display(now)
        return False

    def finish(self,now):
        if self.state['status']=='running':
            self._end(now,'interrupted')
            self.state.update(status='interrupted',end_pc_monotonic_ns=now)
            self.save()
        return self.state['status']
