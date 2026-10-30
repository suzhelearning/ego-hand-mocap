"""Live NatNet connection; bounded queue, explicit loss and source clocks."""
import socket
import struct
import threading
import time
import queue
from natnet_zenoh.natnet_adapter import create_natnet_client, convert_frame


def parse_server_info(packet):
    if len(packet)<4:
        raise ValueError('NatNet ServerInfo header truncated')
    kind,size=struct.unpack_from('<HH',packet)
    if kind!=1 or size<272 or len(packet)<size+4:
        raise ValueError('Invalid or incomplete NatNet ServerInfo')
    if packet[264]<3:
        raise ValueError('NatNet 3+ required for exposure clock frequency')
    frequency=struct.unpack_from('<Q',packet,268)[0]
    if not frequency:
        raise ValueError('NatNet clock frequency is zero')
    return frequency


class NatNetStream:
    def __init__(self, server_ip, client_ip, timeout=15):
        self.server_ip=server_ip;self.client_ip=client_ip;self.timeout=timeout
        self.ready=threading.Event();self.recording=threading.Event()
        self.frames=queue.Queue(maxsize=4096);self.error=None;self.last_received=0
        self.motive_recording=None;self.motive_status_ns=0
        self.names={};self.client=None;self.frequency=None;self.server_packet=None

    def start(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.bind((self.client_ip,0));s.settimeout(4)
            s.sendto(struct.pack('<HH',0,0),(self.server_ip,1510))
            self.server_packet,addr=s.recvfrom(65535)
            if addr[0]!=self.server_ip:raise RuntimeError('Unexpected NatNet response source')
            self.frequency=parse_server_info(self.server_packet)
        self.client=create_natnet_client(self.server_ip,self.client_ip)
        original=self.client._NatNetClient__create_data_socket
        def create_socket():
            s=original()
            if s is not None:s.settimeout(self.timeout+2)
            return s
        self.client._NatNetClient__create_data_socket=create_socket
        self.client.new_frame_with_data_listener=self._on_frame
        self.client.data_descriptions_listener=self._on_names
        if not self.client.run('d'):raise RuntimeError('NatNet start failed')
        if not self.ready.wait(self.timeout):
            raise RuntimeError('未收到NatNet刚体帧，请检查Motive实时模式、刚体输出及网络')
        self.check()

    def _on_names(self,data):
        for b in getattr(data,'rigid_body_list',[]):
            n=getattr(b,'sz_name','');self.names[b.id_num]=n.decode(errors='replace') if isinstance(n,bytes) else str(n)

    def _on_frame(self,data):
        now=time.monotonic_ns()
        try:
            frame=convert_frame(data,now);self.last_received=now
            self.motive_recording=data.get('is_recording')
            self.motive_status_ns=now
            if frame['rigid_bodies']:self.ready.set()
            if self.recording.is_set():self.frames.put_nowait(frame)
        except Exception as e:
            self.error=repr(e);self.ready.set()

    def check(self):
        if self.error:raise RuntimeError('NatNet采集失败: '+self.error)
        if self.last_received and time.monotonic_ns()-self.last_received>3_000_000_000:
            raise RuntimeError('NatNet数据流中断超过3秒')

    def wait_motive_recording(self, expected, since_ns, timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.check()
            if self.motive_status_ns>since_ns and self.motive_recording is expected:return
            time.sleep(.02)
        raise TimeoutError('Motive录制状态未确认: '+str(expected))

    def drain(self):
        result=[]
        for _ in range(4096):
            try:result.append(self.frames.get_nowait())
            except queue.Empty:break
        return result

    def close(self):
        self.recording.clear()
        if self.client:
            self.client.stop_threads=True
            # Wake recvfrom before the SDK joins its worker threads.
            for name in ('data_socket','command_socket'):
                s=getattr(self.client,name,None)
                if s:
                    try:s.shutdown(socket.SHUT_RDWR)
                    except OSError:pass
            self.client.shutdown()
