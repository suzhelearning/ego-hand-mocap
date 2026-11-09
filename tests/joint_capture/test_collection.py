import threading
from types import SimpleNamespace as NS
import pytest
from joint_capture.collection import JointCollection

class Ego:
    def __init__(self,**kw):self.adb=['adb'];self.events=[];self.started=False
    def status(self):return {'boot_id':'boot','control':{'recording':{'active':False}}}
    def start(self):self.started=True;return {'owned':True}
    def stop(self):return {'owned':True}
    def check(self):pass
    def export(self,path):
        import h5py
        path.mkdir();(path/'video').mkdir()
        for name in ('RGB_left','RGB_right'):(path/'video'/(name+'.mp4')).write_bytes(b'original-video')
        with h5py.File(path/'ego.h5','w') as f:
            for name in ('RGB_left','RGB_right'):f['sensor/camera/'+name+'/timestamp_ns']=[1,2]
        return {'h5':str(path/'ego.h5'),'local_directory':str(path),'files':['ego.h5']}
class Stream:
    starts=0
    def __init__(self,*a,**kw):self.frequency=10000000;self.recording=threading.Event();self.motive_recording=False;self.names={};self.server_packet=b''
    def start(self):Stream.starts+=1
    def check(self):pass
    def drain(self):return []
    def close(self):pass
class Probe:
    def __init__(self,kind,path,*a,**kw):self.kind=kind;self.path=path;self.error=None;self.ready=threading.Event();self.frequency=10000000 if kind=='windows' else 1000000000
    def start(self):self.path.write_text('{}\n');self.ready.set()
    def check(self):pass
    def close(self):pass
    def summary(self):return {'path':str(self.path)}
class Take:
    def __init__(self,**kw):self.events=[];self.attempted=False;self.session_id=kw['session_id']
    def prepare(self,stream,start_stream=True):assert not start_stream;self.stream=stream;return {'take_name':self.session_id}
    def start(self):self.attempted=True;self.stream.motive_recording=True;return {'confirmed':True}
    def stop(self):self.stream.motive_recording=False;return {'confirmed':True}
    def check(self):pass
    def verify(self):return {'path':self.session_id+'.tak','size':100,'sha256':'fake'}
    def copy_to(self,path):path.write_bytes(b'raw-take');return dict(self.verify(),local_path=str(path),copied_to_ubuntu=True)

def make(tmp_path,**kw):
    return JointCollection(tmp_path,dict(ego_ip='127.0.0.1',motive_server_ip='127.0.0.1',windows_host='test'),ego_factory=Ego,stream_factory=Stream,probe_factory=Probe,take_factory=kw.get('take_factory',Take),alignment_factory=lambda *a:{'status':'mapped'})

def test_two_actions_keep_stream_and_probes(tmp_path):
    Stream.starts=0;c=make(tmp_path);c.open();probes=list(c.probes)
    try:
        for name in ['a','b']:
            p=tmp_path/name;p.mkdir();c.start_action(p,name);r=c.finish_action()
            assert r['status']=='complete' and r['take']['path']==name+'.tak'
            assert (p/'joint'/r['take']['local_path']).read_bytes()==b'raw-take'
            assert r['take_id']==name
        assert Stream.starts==1 and c.probes==probes
    finally:c.close()

def test_failed_start_is_stopped_and_no_false_complete(tmp_path):
    class Fail(Take):
        stopped=False
        def start(self):super().start();raise RuntimeError('lost ack')
        def stop(self):Fail.stopped=True;return super().stop()
    c=make(tmp_path,take_factory=Fail);c.open();p=tmp_path/'action';p.mkdir()
    try:
        with pytest.raises(RuntimeError,match='lost ack'):c.start_action(p,'failed')
        assert Fail.stopped and c.active is None
        assert (p/'joint'/'manifest.json').exists()
    finally:c.close()

def test_error_during_export_is_not_complete(tmp_path):
    c=make(tmp_path);c.open();p=tmp_path/'action';p.mkdir();c.start_action(p,'a')
    original=c.active['ego'].export
    def export(path):
        result=original(path);c.error='probe disconnected during export';return result
    c.active['ego'].export=export
    try:assert c.finish_action()['status']=='failed'
    finally:c.close()

def test_slow_ego_check_does_not_block_stop_or_drain(tmp_path):
    import time
    c=make(tmp_path);c.open();p=tmp_path/'action';p.mkdir();c.start_action(p,'a')
    entered=threading.Event();release=threading.Event()
    def check():entered.set();release.wait(3)
    c.active['ego'].check=check
    try:
        assert entered.wait(2)
        acquired=c.lock.acquire(timeout=.1)
        if acquired:c.lock.release()
        assert acquired
    finally:release.set();c.close()
