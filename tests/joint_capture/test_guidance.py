import json
import threading
import pytest
from joint_capture.cli import parser
from joint_capture.guidance import CalibrationGuide, build_plan
from joint_capture.session import run_session
from test_session import config, FakeEgo, FakeStream


def test_cli_calibration_selects_h5_and_normalizes_case():
    args=parser().parse_args(['--calibration','RBC'])
    assert args.mode=='h5' and args.calibration=='rbc'
    assert parser().parse_args([]).mode=='take'


@pytest.mark.parametrize('extra',[['--mode','take'],['--duration','10'],['--no-clock-probes']])
def test_guided_recording_rejects_incompatible_options(extra):
    with pytest.raises(SystemExit):parser().parse_args(['--calibration','tbc',*extra])


def test_actual_boundaries_and_interruption_are_persisted(tmp_path):
    events=[];output=[]
    plan=[dict(id='first',instruction='保持静止',duration_s=2,kind='hold'),dict(id='second',instruction='向左转',duration_s=3,kind='move')]
    guide=CalibrationGuide('rbc',tmp_path/'guide.json',lambda name,**kw:events.append((name,kw)),output.append,plan=plan)
    guide.start(1000000000)
    assert not guide.tick(2000000000)
    assert '1 秒' in output[-1]
    # Delayed loop does not pretend the next instruction began in the past.
    assert not guide.tick(5000000000)
    guide.finish(5500000000)
    saved=json.loads((tmp_path/'guide.json').read_text())
    assert saved['status']=='interrupted'
    assert saved['segments'][0]['end_pc_monotonic_ns']==5000000000
    assert saved['segments'][1]['start_pc_monotonic_ns']==5000000000
    assert saved['segments'][1]['status']=='interrupted'
    assert len([e for e in events if e[0]=='calibration_step_start'])==2


def test_plan_covers_three_axes_and_three_tbc_locations():
    for target in ['rbc','tbc']:
        plan=build_plan(target)
        assert {'yaw','pitch','roll'} <= {s.get('axis') for s in plan}
        assert all(s['duration_s']>0 for s in plan)
        assert len({s['id'] for s in plan})==len(plan)
    assert {'center','left_front','right_front'} <= {s.get('location') for s in build_plan('tbc')}


def test_completed_guide_auto_stops_and_exports(tmp_path,monkeypatch):
    import joint_capture.guidance as guidance
    monkeypatch.setattr(guidance,'build_plan',lambda target:[dict(id='test',instruction='保持静止',kind='hold',duration_s=.01)])
    ego=FakeEgo()
    result=run_session(config(tmp_path,session_id=None,duration=None,calibration='rbc'),threading.Event(),ego_factory=lambda **kw:ego,stream_factory=FakeStream)
    assert result['status']=='complete' and ego.stopped
    assert result['session_id'].startswith('calibration_rbc_')
    path=tmp_path/'data/sessions'/result['session_id']
    saved=json.loads((path/'capture/calibration_guide.json').read_text())
    assert saved['status']=='complete'
    assert result['calibration']['status']=='complete'
    assert 'calibration_guide' in result['assets']


def test_interrupted_guide_keeps_data_but_not_complete(tmp_path):
    stop=threading.Event()
    class StoppingEgo(FakeEgo):
        def start(self):
            value=super().start();stop.set();return value
    result=run_session(config(tmp_path,duration=None,calibration='tbc'),stop,ego_factory=StoppingEgo,stream_factory=FakeStream)
    assert result['status']=='failed'
    assert result['calibration']['status']=='interrupted'
    assert 'ego_h5' in result['assets']
