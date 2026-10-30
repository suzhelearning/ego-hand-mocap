import socket
import json
import struct
import threading
import pytest
from joint_capture.motive import NatNetCommands, validate_take_root


def test_command_protocol_roundtrip():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as server:
        server.bind(('127.0.0.1',0));server.settimeout(2)
        received=[]
        def respond():
            packet,peer=server.recvfrom(1024);received.append(packet)
            server.sendto(struct.pack('<HHi',3,4,0),peer)
        thread=threading.Thread(target=respond);thread.start()
        client=NatNetCommands('127.0.0.1','127.0.0.1',timeout=1,port=server.getsockname()[1])
        assert client.mode()==0
        thread.join()
        assert received==[struct.pack('<HH',2,12)+b'CurrentMode\0']


@pytest.mark.parametrize('path',['relative','D:relative','D:\\a,b','D:\\bad\npath','D:\\a"b'])
def test_invalid_windows_root(path):
    with pytest.raises(ValueError):validate_take_root(path)


def test_windows_absolute_root():
    assert str(validate_take_root('D:\\Mocap Captures'))=='D:\\Mocap Captures'

from joint_capture.motive import MotiveTakeController


class Stream:
    def __init__(self,recording=False):self.motive_recording=recording
    def start(self):pass
    def check(self):pass
    def wait_motive_recording(self,expected,since,timeout):
        if self.motive_recording is not expected:raise TimeoutError('state unchanged')


class Commands:
    def __init__(self,stream):self.stream=stream;self.values=[];self.path='D:\\captures';self.current_mode=0
    def mode(self):return self.current_mode
    def session_path(self):return self.path
    def command(self,value):
        self.values.append(value)
        if value=='LiveMode':self.current_mode=0
        if value=='StartRecording':self.stream.motive_recording=True
        if value=='StopRecording':self.stream.motive_recording=False


def controller(stream,monkeypatch):
    take=MotiveTakeController('127.0.0.1','127.0.0.1','windows-pc','D:\\captures','test',timeout=.1)
    take.commands=Commands(stream)
    monkeypatch.setattr(take,'_powershell',lambda *args,**kwargs:'')
    return take


def test_existing_take_recording_is_not_stopped(monkeypatch):
    stream=Stream(True);take=controller(stream,monkeypatch)
    with pytest.raises(RuntimeError,match='拒绝接管'):take.prepare(stream)
    take.stop()
    assert stream.motive_recording is True
    assert not take.attempted


def test_recording_started_during_destination_check_is_not_taken_over(monkeypatch):
    stream = Stream()
    take = controller(stream, monkeypatch)
    take.prepare(stream)

    def operator_starts_recording():
        stream.motive_recording = True

    monkeypatch.setattr(take, '_check_destination_absent', operator_starts_recording)
    with pytest.raises(RuntimeError, match='拒绝启动'):
        take.start()
    take.stop()
    assert not take.attempted
    assert stream.motive_recording is True
    assert 'StartRecording' not in take.commands.values
    assert 'StopRecording' not in take.commands.values


def test_take_lifecycle_confirms_state(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch)
    take.commands.current_mode=1
    take.prepare(stream);take.start();take.check()
    assert stream.motive_recording is True
    take.stop()
    assert take.stop_confirmed and stream.motive_recording is False
    assert take.commands.values==['LiveMode','SetCurrentSession,D:\\captures',
        'SetRecordTakeName,'+take.take_name,'StartRecording','StopRecording']


def test_wrong_session_directory_rejects_start(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch);take.commands.path='D:\\other'
    with pytest.raises(RuntimeError,match='目录确认失败'):take.prepare(stream)
    assert not take.attempted and stream.motive_recording is False


def test_take_rejects_premature_stop(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch)
    take.prepare(stream);take.start();stream.motive_recording=False
    with pytest.raises(RuntimeError,match='提前停止'):take.check()


def test_take_file_verification_requires_confirmed_stop(monkeypatch):
    take=controller(Stream(),monkeypatch)
    with pytest.raises(RuntimeError,match='停止未确认'):take.verify()


def test_natnet_state_wait_requires_fresh_frame():
    from joint_capture.natnet import NatNetStream
    stream=NatNetStream('127.0.0.1','127.0.0.1')
    stream.motive_recording=True;stream.motive_status_ns=10
    with pytest.raises(TimeoutError):stream.wait_motive_recording(True,20,.01)
    stream.motive_status_ns=30
    stream.wait_motive_recording(True,20,.01)


def test_stop_retries_a_lost_udp_request(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch)
    take.prepare(stream);take.start()
    original=take.commands.command;attempts=[]
    def lossy(value):
        attempts.append(value)
        if len(attempts)==1:raise TimeoutError('UDP packet lost')
        return original(value)
    monkeypatch.setattr(take.commands,'command',lossy)
    assert take.stop()['confirmed']
    assert len(attempts)==2 and stream.motive_recording is False


def test_stop_accepts_state_confirmation_after_lost_ack(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch)
    take.prepare(stream);take.start()
    def lost_ack(value):
        stream.motive_recording=False
        raise TimeoutError('UDP reply lost')
    monkeypatch.setattr(take.commands,'command',lost_ack)
    assert take.stop()['confirmed']


def test_stop_failure_is_bounded(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch)
    take.prepare(stream);take.start();attempts=[]
    def lost(value):attempts.append(value);raise TimeoutError('lost')
    monkeypatch.setattr(take.commands,'command',lost)
    with pytest.raises(RuntimeError,match='停止'):
        take.stop()
    assert len(attempts)==3
    assert not take.stop_confirmed


def test_default_take_root_is_resolved_on_windows(monkeypatch):
    stream=Stream()
    take=MotiveTakeController('127.0.0.1','127.0.0.1','windows-pc',None,'test',timeout=.1)
    take.commands=Commands(stream)
    take.commands.path=r'C:\Users\Windows User\Desktop\syz\ego-hand\remote'
    calls=[]
    def remote(code,**kwargs):
        calls.append(code)
        return json.dumps(r'C:\Users\Windows User') if 'UserProfile' in code else ''
    monkeypatch.setattr(take,'_powershell',remote)
    result=take.prepare(stream)
    assert result['session_directory']==r'C:\Users\Windows User\Desktop\syz\ego-hand\remote'
    assert any('UserProfile' in code for code in calls)


@pytest.mark.parametrize('returncode',[0,1])
def test_powershell_handles_windows_encoded_stderr(monkeypatch,returncode):
    import subprocess
    import sys
    original=subprocess.run
    diagnostic='正在准备首次使用模块。' if returncode==0 else '无法创建目录：拒绝访问。'
    output=json.dumps('C:\\Users\\测试',ensure_ascii=False)
    program=(f'import sys; sys.stdout.buffer.write({output.encode("utf-8")!r}); '
             f'sys.stderr.buffer.write({diagnostic.encode("gb18030")!r}); sys.exit({returncode})')
    def run(args,**kwargs):return original([sys.executable,'-c',program],**kwargs)
    monkeypatch.setattr(subprocess,'run',run)
    take=MotiveTakeController('127.0.0.1','127.0.0.1','windows-pc',None,'test')
    if returncode:
        with pytest.raises(RuntimeError,match='拒绝访问'):take._powershell('test')
    else:
        assert take._powershell('test')==output


@pytest.mark.parametrize('status',[0,1,-1])
def test_command_status_with_trailing_server_bytes(monkeypatch,status):
    client=NatNetCommands('127.0.0.1','127.0.0.1')
    monkeypatch.setattr(client,'request',lambda command:struct.pack('<i',status)+b'254.1.1:37067\0\xffMarker 001\0')
    if status==0:client.command('SetCurrentSession,C:\\captures')
    else:
        with pytest.raises(RuntimeError):client.command('SetCurrentSession,C:\\captures')


def test_session_path_ends_at_first_null(monkeypatch):
    client=NatNetCommands('127.0.0.1','127.0.0.1')
    monkeypatch.setattr(client,'request',lambda command:b'C:/captures/\0\xffunused')
    assert client.session_path()=='C:/captures/'


def test_session_directory_switch_can_be_async(monkeypatch):
    stream=Stream();take=controller(stream,monkeypatch)
    paths=iter(['D:\\previous', 'D:\\captures'])
    monkeypatch.setattr(take.commands,'session_path',lambda:next(paths))
    assert take.prepare(stream)['session_directory']=='D:\\captures'


def test_take_filename_is_timestamp_not_session_id(monkeypatch):
    import re
    take=controller(Stream(),monkeypatch)
    assert re.fullmatch(r'\d{8}_\d{6}_\d{6}',take.take_name)
    assert str(take.directory)=='D:\\captures'


def test_verify_rejects_another_take_in_shared_directory(monkeypatch):
    take=controller(Stream(),monkeypatch);take.stop_confirmed=True
    monkeypatch.setattr(take,'_powershell',lambda *args,**kw:json.dumps({
        'path':'D:\\captures\\other.tak','size':10,'sha256':'a'*64}))
    with pytest.raises(ValueError,match='错误文件'):take.verify()
