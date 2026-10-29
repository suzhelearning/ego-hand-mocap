import socket
import threading
import pytest
from joint_capture.clock import sample_exchange


def test_clock_exchange_retains_remote_ticks_and_pc_envelope():
    host,remote=socket.socketpair()
    def respond():
        with remote,remote.makefile('rb') as f:
            assert f.readline()==b'7\n'
            remote.sendall(b'7 100 102\n')
    t=threading.Thread(target=respond);t.start()
    with host,host.makefile('rb') as stream:
        result=sample_exchange(host,stream,7)
    t.join(2)
    assert result['remote_receive_ticks']==100
    assert result['remote_send_ticks']==102
    assert result['pc_send_ns']<=result['pc_receive_ns']


def test_clock_exchange_rejects_wrong_sequence():
    host,remote=socket.socketpair();remote.sendall(b'99 100 102\n')
    try:
        with host.makefile('rb') as stream:
            with pytest.raises(ValueError,match='sequence'):sample_exchange(host,stream,7)
    finally:host.close();remote.close()


def test_probe_process_isolated_from_terminal_sigint(tmp_path, monkeypatch):
    import joint_capture.clock as clock
    seen={}
    def launch(*args,**kwargs):
        seen.update(kwargs)
        raise RuntimeError('stop before external connection')
    monkeypatch.setattr(clock.subprocess,'Popen',launch)
    monkeypatch.setattr(clock,'local_address',lambda ip:'127.0.0.1')
    probe=clock.ClockProbe('ego',tmp_path/'probe.jsonl','127.0.0.1',adb=['adb'])
    probe._run()
    assert seen.get('start_new_session') is True


def test_natnet_idle_socket_timeout_keeps_receiving():
    from natnet_zenoh._sdk.NatNetClient import NatNetClient
    client=NatNetClient();calls=[]
    class IdleSocket:
        def recvfrom(self,size):
            calls.append(size)
            raise socket.timeout('idle')
    client._NatNetClient__data_thread_function(IdleSocket(),lambda:len(calls)>=2,lambda:0)
    assert len(calls)==2
