from types import SimpleNamespace
import threading
import json
import h5py
import pytest
from joint_capture.session import run_session
from joint_capture.cli import parser
from test_storage import frame, body


def config(root, **kwargs):
    values=dict(output_root=root,session_id='test-session',duration=.01,ego_serial=None,
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
    assert not (tmp_path/'data/sessions/test-session/raw/mocap/natnet.h5').exists()


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
    assert result['mocap']['file']=='raw/mocap/natnet.h5'
    assert result['ego']['local_directory']=='raw/ego'
    base=tmp_path/'data/sessions/test-session'
    for asset in result['assets'].values():
        assert (base/asset['path']).is_file()
    assert not (tmp_path/'ego_data').exists()
