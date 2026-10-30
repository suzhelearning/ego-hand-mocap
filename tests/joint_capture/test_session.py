from types import SimpleNamespace
import threading
import json
import h5py
import pytest
from joint_capture.session import run_session
from joint_capture.cli import parser
from test_storage import frame, body


def config(root, **kwargs):
    values=dict(mode='h5',output_root=root,session_id='test-session',duration=.01,ego_serial=None,
        motive_server_ip='127.0.0.1',natnet_client_ip='127.0.0.1',ego_ip='127.0.0.1',
        windows_host='windows-pc',no_clock_probes=True,startup_timeout=1,warmup=0,tail=0)
    return SimpleNamespace(**dict(values,**kwargs))


class FakeStream:
    def __init__(self,*args,**kwargs):
        self.frequency=10000000;self.server_packet=b'test';self.recording=threading.Event();self.names={1:'ego'};self.n=0
    def start(self):pass
    def check(self):pass
    def drain(self):
        if not self.recording.is_set():return []
        self.n+=1;return [frame(self.n,[body(1)])]
    def close(self):self.recording.clear()


class FakeEgo:
    active=False
    def __init__(self,*args,**kwargs):self.events=[];self.adb=['adb'];self.started=False;self.stopped=False
    def status(self):return {'control':{'recording':{'active':self.active}},'boot_id':'fixture'}
    def check(self):pass
    def start(self):self.started=True;return {'owned':True}
    def stop(self):self.stopped=True;return {'owned':True}
    def export(self,path):
        path.mkdir(parents=True)
        p=path/'head_1.h5'
        with h5py.File(p,'w') as f:f.create_dataset('sensor/camera/RGB_left/timestamp_ns',data=[100,200])
        return {'h5':str(p),'local_directory':str(path)}


def test_session_saves_both_h5_and_manifest(tmp_path):
    ego=FakeEgo()
    result=run_session(config(tmp_path),threading.Event(),ego_factory=lambda **kw:ego,stream_factory=FakeStream)
    assert result['status']=='complete' and ego.started and ego.stopped
    assert result['mocap']['frames']>0
    assert result['ego']['rgb_frames']==2
    with h5py.File(tmp_path/'data/sessions/test-session'/result['mocap']['file']) as f:assert f.attrs['complete']
    manifest=json.loads((tmp_path/'data/sessions/test-session/session.json').read_text())
    assert manifest['physical_exposure_calibrated'] is False


def test_existing_recording_is_not_touched(tmp_path):
    ego=FakeEgo();ego.active=True
    result=run_session(config(tmp_path),threading.Event(),ego_factory=lambda **kw:ego,stream_factory=FakeStream)
    assert result['status']=='failed'
    assert not ego.started and not ego.stopped
    assert not (tmp_path/'data/sessions/test-session/mocap.h5').exists()


def test_disconnect_keeps_partial_data_and_stops_owned_ego(tmp_path):
    class BrokenStream(FakeStream):
        def check(self):
            if self.n:raise RuntimeError('stream disconnected')
    ego=FakeEgo()
    result=run_session(config(tmp_path,duration=.2),threading.Event(),ego_factory=lambda **kw:ego,stream_factory=BrokenStream)
    assert result['status']=='failed' and ego.stopped
    with h5py.File(tmp_path/'data/sessions/test-session'/result['mocap']['file']) as f:
        assert len(f['frames/frame_number'])>=1
        assert not f.attrs['complete']
    assert result['ego']['rgb_frames']==2


def test_duplicate_session_preserves_previous_files(tmp_path):
    root=tmp_path/'capture_sessions/test-session';root.mkdir(parents=True);(root/'session.json').write_text('original')
    with pytest.raises(FileExistsError):run_session(config(tmp_path),threading.Event())
    assert (root/'session.json').read_text()=='original'


@pytest.mark.parametrize('value',['0','-1','nan','inf'])
def test_invalid_duration_rejected_before_connect(value):
    with pytest.raises(SystemExit):parser().parse_args(['--duration',value])


def test_premature_ego_stop_never_reports_complete(tmp_path):
    class EarlyStop(FakeEgo):
        def stop(self):
            self.stopped=True
            return {'owned':True,'stopped':True,'premature':True}
    ego=EarlyStop()
    result=run_session(config(tmp_path),threading.Event(),ego_factory=lambda **kw:ego,stream_factory=FakeStream)
    assert result['status']=='failed'
    assert result['ego']['rgb_frames']==2


def test_manifest_write_failure_still_stops_devices(tmp_path, monkeypatch):
    import joint_capture.session as session
    original=session.save_json
    def fail_saving(path,data):
        if isinstance(data,dict) and data.get('status')=='saving':
            raise OSError('disk full')
        original(path,data)
    monkeypatch.setattr(session,'save_json',fail_saving)
    ego=FakeEgo()
    result=run_session(config(tmp_path),threading.Event(),ego_factory=lambda **kw:ego,stream_factory=FakeStream)
    assert ego.stopped and result['status']=='failed'


def test_capture_uses_project_data_directory(tmp_path):
    result=run_session(config(tmp_path),threading.Event(),ego_factory=FakeEgo,stream_factory=FakeStream)
    assert result['status']=='complete'
    assert result['capture_config']['export_timeout_s']==3600
    assert result['mocap']['file']=='mocap.h5'
    assert result['ego']['local_directory']=='.'
    base=tmp_path/'data/sessions/test-session'
    for asset in result['assets'].values():
        assert (base/asset['path']).is_file()
    assert not (tmp_path/'ego_data').exists()

class FakeTake:
    def __init__(self, *args, **kwargs):self.events=[];self.attempted=False;self.stopped=False
    def prepare(self, stream):self.stream=stream;return {'session_directory':'D:\\captures\\test-session'}
    def start(self):self.attempted=True;return {'confirmed':True}
    def check(self):pass
    def stop(self):self.stopped=True;return {'confirmed':True}
    def verify(self):return {'path':'D:\\captures\\test-session\\test-session.tak','size':123,'sha256':'a'*64}


def test_take_mode_does_not_write_mocap_h5(tmp_path):
    take=FakeTake()
    result=run_session(config(tmp_path,mode='take',windows_take_root='D:\\captures'),
        threading.Event(),ego_factory=FakeEgo,stream_factory=FakeStream,take_factory=lambda **kw:take)
    assert result['status']=='complete'
    assert result['capture_mode']=='take'
    assert take.stopped
    assert 'file' not in result['mocap'] and 'mocap_h5' not in result['assets']
    assert result['remote_assets']['motive_take']['size']==123
    assert not list((tmp_path/'data/sessions').rglob('natnet.h5'))


def test_take_start_failure_still_attempts_stop(tmp_path):
    class BrokenTake(FakeTake):
        def start(self):self.attempted=True;raise TimeoutError('ack lost')
    take=BrokenTake();ego=FakeEgo()
    result=run_session(config(tmp_path,mode='take',windows_take_root='D:\\captures'),
        threading.Event(),ego_factory=lambda **kw:ego,stream_factory=FakeStream,take_factory=lambda **kw:take)
    assert result['status']=='failed' and take.stopped
    assert not ego.started


def test_take_mode_uses_default_windows_root(tmp_path):
    received={}
    def factory(**kwargs):received.update(kwargs);return FakeTake()
    result=run_session(config(tmp_path,mode='take'),threading.Event(),
        ego_factory=FakeEgo,stream_factory=FakeStream,take_factory=factory)
    assert result['status']=='complete'
    assert received['take_root']=='~/Desktop/syz/ego-hand/remote'
    assert result['capture_config']['windows_take_root']=='~/Desktop/syz/ego-hand/remote'


def test_cli_two_modes_only():
    assert parser().parse_args([]).mode=='take'
    args=parser().parse_args(['--mode','take','--windows-take-root','D:\\captures'])
    assert args.mode=='take'
    with pytest.raises(SystemExit):parser().parse_args(['--mode','both'])

@pytest.mark.parametrize('failure',['stop','verify'])
def test_take_failure_preserves_ego_and_reports_failed(tmp_path,failure):
    take=FakeTake()
    def broken():raise RuntimeError('injected '+failure)
    setattr(take,failure,broken)
    result=run_session(config(tmp_path,mode='take',windows_take_root='D:\\captures'),
        threading.Event(),ego_factory=FakeEgo,stream_factory=FakeStream,take_factory=lambda **kw:take)
    assert result['status']=='failed'
    assert 'ego_h5' in result['assets']
    assert any(failure in error['error'] for error in result['errors'])


def test_h5_mode_never_controls_motive_recording(tmp_path):
    def unexpected(**kwargs):raise AssertionError('Take controller used in H5 mode')
    result=run_session(config(tmp_path),threading.Event(),ego_factory=FakeEgo,
        stream_factory=FakeStream,take_factory=unexpected)
    assert result['status']=='complete'
    assert result['capture_mode']=='h5'
    assert 'remote_assets' not in result


def test_cancellation_after_take_start_does_not_start_ego(tmp_path):
    stop=threading.Event();ego=FakeEgo()
    class CancellingTake(FakeTake):
        def start(self):
            result=super().start();stop.set();return result
    take=CancellingTake()
    result=run_session(config(tmp_path,mode='take',windows_take_root='D:\\captures'),
        stop,ego_factory=lambda **kw:ego,stream_factory=FakeStream,take_factory=lambda **kw:take)
    assert not ego.started
    assert take.stopped and result['status']=='failed'


def test_flat_capture_places_ego_and_mocap_at_session_root(tmp_path):
    result=run_session(config(tmp_path),threading.Event(),ego_factory=FakeEgo,stream_factory=FakeStream)
    base=tmp_path/'data/sessions/test-session'
    assert result['status']=='complete'
    assert (base/'ego.h5').is_file() and (base/'mocap.h5').is_file()
    assert not (base/'raw').exists() and not (base/'runs').exists()
    assert result['assets']['ego_h5']['path']=='ego.h5'
    assert result['assets']['mocap_h5']['path']=='mocap.h5'


def test_h5_default_session_is_named_calibration_and_printed(tmp_path, capsys):
    result=run_session(config(tmp_path,session_id=None),threading.Event(),
        ego_factory=FakeEgo,stream_factory=FakeStream)
    assert result['session_id'].startswith('calibration_')
    assert result['capture_purpose']=='ego_extrinsic_calibration'
    folder=tmp_path/'data/sessions'/result['session_id']
    assert (folder/'ego.h5').is_file()
    assert (folder/'mocap.h5').is_file()
    assert str(folder.resolve()) in capsys.readouterr().out
