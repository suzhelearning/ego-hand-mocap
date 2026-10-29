"""Continuous four-timestamp probes; saves evidence, does not set any clock."""
import base64
import json
from pathlib import Path
import shlex
import socket
import subprocess
import threading
import time
import uuid


def local_address(target):
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.connect((target,9));return s.getsockname()[0]


def sample_exchange(conn,stream,index):
    t1=time.monotonic_ns();conn.sendall((str(index)+'\n').encode())
    line=stream.readline(256);t4=time.monotonic_ns()
    if not line:raise ConnectionError('Clock probe disconnected')
    seq,t2,t3=map(int,line.split())
    if seq!=index:raise ValueError('Clock probe sequence mismatch')
    if t2<0 or t3<t2:raise ValueError('Invalid remote clock timestamps')
    return dict(index=index,pc_send_ns=t1,remote_receive_ticks=t2,
                remote_send_ticks=t3,pc_receive_ns=t4)


class ClockProbe:
    def __init__(self, kind, path, remote_ip, adb=None, ssh_host='windows-pc', interval=.2):
        self.kind=kind;self.path=Path(path);self.remote_ip=remote_ip;self.adb=adb
        self.ssh_host=ssh_host;self.interval=interval
        self.ready=threading.Event();self.stopped=threading.Event()
        self.error=None;self.count=0;self.frequency=None;self.thread=None

    def _command(self,host,port,nonce):
        if self.kind=='ego':
            code=f'''import socket,time
s=socket.create_connection(({host!r},{port}),timeout=10);s.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
s.sendall({(nonce+' 1000000000'+chr(10)).encode()!r})
with s,s.makefile("rb") as stream:
 while True:
  line=stream.readline();t2=time.monotonic_ns()
  if not line or line.strip()==b"stop":break
  i=int(line);t3=time.monotonic_ns();s.sendall((str(i)+" "+str(t2)+" "+str(t3)+"\\n").encode())
'''
            return [*self.adb,'shell','python3 -u -c '+shlex.quote(code)]
        code=f'''$ErrorActionPreference="Stop"; $ProgressPreference="SilentlyContinue"; $c=New-Object Net.Sockets.TcpClient; $c.Connect("{host}",{port}); $c.NoDelay=$true; $s=$c.GetStream(); $s.ReadTimeout=10000; $r=New-Object IO.StreamReader($s); $w=New-Object IO.StreamWriter($s); $w.AutoFlush=$true; try {{ $w.WriteLine("{nonce} " + [Diagnostics.Stopwatch]::Frequency); while($true) {{ $line=$r.ReadLine(); if($null -eq $line -or $line -eq "stop"){{break}}; $t2=[Diagnostics.Stopwatch]::GetTimestamp(); $t3=[Diagnostics.Stopwatch]::GetTimestamp(); $w.WriteLine("$line $t2 $t3") }} }} finally {{ $c.Close() }}'''
        encoded=base64.b64encode(code.encode('utf-16le')).decode()
        return ['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5',self.ssh_host,
                'powershell -NoProfile -EncodedCommand '+encoded]

    def start(self):
        self.thread=threading.Thread(target=self._run,name=self.kind+'-clock',daemon=True)
        self.thread.start()

    def _run(self):
        process=None
        try:
            with self.path.open('x',encoding='utf8') as output, socket.socket() as listener:
                host=local_address(self.remote_ip);listener.bind((host,0));listener.listen(1);listener.settimeout(20)
                nonce=uuid.uuid4().hex
                with self.path.with_suffix('.stderr.log').open('wb') as log:
                    process=subprocess.Popen(self._command(host,listener.getsockname()[1],nonce),
                        stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
                    conn,addr=listener.accept();conn.settimeout(5);conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    with conn,conn.makefile('rb') as stream:
                        token,freq=stream.readline(256).decode().split();self.frequency=int(freq)
                        if token!=nonce or addr[0]!=self.remote_ip or self.frequency<=0:
                            raise ValueError('Unexpected clock probe peer or greeting')
                        output.write(json.dumps({'type':'metadata','remote':self.kind,'remote_ip':addr[0],
                            'remote_frequency_hz':self.frequency,'pc_time_domain':'CLOCK_MONOTONIC',
                            'physical_exposure_calibrated':False})+'\n');output.flush()
                        while not self.stopped.is_set():
                            row=sample_exchange(conn,stream,self.count)
                            output.write(json.dumps(row)+'\n');output.flush()
                            self.count+=1;self.ready.set()
                            self.stopped.wait(self.interval)
                        conn.sendall(b'stop\n')
        except Exception as e:self.error=repr(e);self.ready.set()
        finally:
            if process:
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                if process.returncode and not self.error:self.error='Remote probe exited '+str(process.returncode)

    def check(self):
        if self.error:raise RuntimeError(self.kind+' clock: '+self.error)

    def close(self):
        self.stopped.set()
        if self.thread:
            self.thread.join(timeout=30)
            if self.thread.is_alive():self.error='Clock worker did not finish'

    def summary(self):
        return {'samples':self.count,'frequency_hz':self.frequency,'error':self.error,'path':str(self.path)}
