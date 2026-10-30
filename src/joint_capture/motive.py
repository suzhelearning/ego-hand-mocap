"""Motive Take control over NatNet; Windows filesystem verification over SSH."""
from datetime import datetime
from zoneinfo import ZoneInfo
import base64
import json
from pathlib import PureWindowsPath
import socket
import struct
import subprocess
import time


DEFAULT_TAKE_ROOT="~/Desktop/syz/ego-hand/remote"

def validate_take_root(value):
    if value is None:value=DEFAULT_TAKE_ROOT
    if value==DEFAULT_TAKE_ROOT:return PureWindowsPath(value)
    path=PureWindowsPath(value)
    if not path.is_absolute() or any(c in str(value) for c in ',\r\n\0"*?<>|') or '..' in path.parts:
        raise ValueError('--windows-take-root 必须是无逗号、通配符的Windows绝对目录')
    return path


class NatNetCommands:
    def __init__(self, server_ip, client_ip, timeout=15, port=1510):
        self.server_ip=server_ip;self.client_ip=client_ip;self.timeout=timeout;self.port=port
        self.events=[]

    def request(self, command):
        payload=command.encode('utf8')+b'\0'
        event={'command':command,'pc_send_ns':time.monotonic_ns()};self.events.append(event)
        # One socket per request prevents a late reply being accepted for the next command.
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                sock.bind((self.client_ip,0));sock.connect((self.server_ip,self.port))
                deadline=time.monotonic()+self.timeout
                sock.send(struct.pack('<HH',2,len(payload))+payload)
                while True:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise TimeoutError('Motive命令应答超时: '+command)
                    sock.settimeout(remaining);packet=sock.recv(65535)
                    if len(packet)<4:raise ValueError('Motive应答头不完整')
                    kind,size=struct.unpack_from('<HH',packet)
                    if kind==100:raise RuntimeError('Motive不支持命令: '+command)
                    if kind!=3:continue
                    if len(packet)!=size+4:raise ValueError('Motive应答长度无效')
                    event['pc_receive_ns']=time.monotonic_ns()
                    event['response_bytes']=size
                    event['response_prefix_hex']=packet[4:20].hex()
                    return packet[4:]
        except Exception as exc:
            event['error']=str(exc);raise

    def mode(self):
        data=self.request('CurrentMode')
        if len(data)!=4:raise ValueError('CurrentMode应答无效')
        mode=struct.unpack('<i',data)[0]
        if mode not in (0,1):raise ValueError('未知Motive模式')
        return mode

    def command(self, value):
        result=self.request(value)
        # Some Motive versions append stale buffer bytes after the 32-bit status.
        # Accept only zero status; callers separately confirm directory/recording state.
        if len(result)>=4:
            status=struct.unpack_from('<i',result)[0]
            if status==0:return
            raise RuntimeError('Motive拒绝命令: '+value+'; status='+str(status))
        if result not in (b'',b'\0'):
            raise RuntimeError('Motive命令应答无效: '+value+'; response='+repr(result))

    def session_path(self):
        return self.request('CurrentSessionPath').split(b'\0',1)[0].decode('utf8')


class MotiveTakeController:
    def __init__(self, server_ip, client_ip, windows_host, take_root, session_id,
                 timeout=15, export_timeout=3600):
        self.commands=NatNetCommands(server_ip,client_ip,timeout)
        self.events=self.commands.events
        self.windows_host=windows_host
        self.take_root=validate_take_root(take_root)
        self.directory=self.take_root
        self.take_name=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        self.session_id=session_id;self.timeout=timeout;self.export_timeout=export_timeout
        self.stream=None;self.attempted=False;self.stop_confirmed=False

    def _powershell(self, code, timeout=None):
        preamble=('$ErrorActionPreference="Stop"; $ProgressPreference="SilentlyContinue"; '
                  '[Console]::OutputEncoding=New-Object System.Text.UTF8Encoding($false); '
                  '$OutputEncoding=[Console]::OutputEncoding; ')
        encoded=base64.b64encode((preamble+code).encode('utf-16le')).decode()
        result=subprocess.run(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5',self.windows_host,
            'powershell -NoProfile -EncodedCommand '+encoded],capture_output=True,
            timeout=timeout or self.timeout,start_new_session=True)
        if result.returncode:
            try:diagnostic=result.stderr.decode('utf-8-sig')
            except UnicodeDecodeError:diagnostic=result.stderr.decode('gb18030',errors='replace')
            raise RuntimeError('Windows文件操作失败: '+diagnostic[-2000:])
        # stdout is our UTF-8 protocol; successful stderr may contain legacy CLIXML progress.
        return result.stdout.decode('utf-8-sig').strip()

    def _path_literal(self):
        return "'"+str(self.directory).replace("'","''")+"'"

    def prepare(self, stream):
        self.stream=stream
        if self.take_root.parts[0]=='~':
            home=json.loads(self._powershell('[Environment]::GetFolderPath("UserProfile") | ConvertTo-Json -Compress'))
            resolved=validate_take_root(str(PureWindowsPath(home)/PureWindowsPath(*self.take_root.parts[1:])))
            self.directory=resolved
        if self.commands.mode()!=0:
            self.commands.command('LiveMode')
            if self.commands.mode()!=0:raise RuntimeError('Motive未进入Live模式')
        stream.start()
        if stream.motive_recording is not False:raise RuntimeError('Motive已在录制或状态未知，拒绝接管')
        # Reuse the root; only this recording's timestamp-named file must be absent.
        self._powershell('$p='+self._path_literal()+'; '
            'if(!(Test-Path -LiteralPath $p)){New-Item -ItemType Directory -Path $p -ErrorAction Stop | Out-Null}; '
            'if(!(Test-Path -LiteralPath $p -PathType Container)){throw "Take root is not a directory"}')
        self._check_destination_absent()
        self.commands.command('SetCurrentSession,'+str(self.directory))
        deadline=time.monotonic()+self.timeout
        while PureWindowsPath(self.commands.session_path())!=self.directory:
            if time.monotonic()>=deadline:raise RuntimeError('Motive会话保存目录确认失败')
            time.sleep(.05)
        self.commands.command('SetRecordTakeName,'+self.take_name)
        return {'session_directory':str(self.directory),'take_name':self.take_name,'host':self.windows_host}

    def _check_destination_absent(self):
        target=str(self.directory/(self.take_name+'.tak')).replace("'","''")
        self._powershell("if(Test-Path -LiteralPath '"+target+"'){throw 'Take file already exists'}")

    def start(self):
        self._check_destination_absent()
        # SSH can block while an operator starts a Take; recheck before taking ownership.
        self.stream.check()
        if self.stream.motive_recording is not False:raise RuntimeError('Motive录制状态已变化，拒绝启动')
        self.attempted=True
        since=time.monotonic_ns()
        self.commands.command('StartRecording')
        self.stream.wait_motive_recording(True,since,self.timeout)
        return {'confirmed':True,'pc_confirmed_ns':time.monotonic_ns()}

    def check(self):
        self.stream.check()
        if self.stream.motive_recording is not True:raise RuntimeError('Motive录制提前停止')

    def stop(self):
        if not self.attempted:return {'confirmed':False}
        last_error=None
        for attempt in range(3):
            since=time.monotonic_ns()
            try:
                self.commands.command('StopRecording')
            except Exception as exc:
                # An acknowledgement may be lost even though recording stopped.
                last_error=exc
            try:
                self.stream.wait_motive_recording(False,since,self.timeout)
                self.stop_confirmed=True
                return {'confirmed':True,'attempts':attempt+1,'pc_confirmed_ns':time.monotonic_ns()}
            except Exception as exc:
                last_error=exc
        raise RuntimeError('Motive停止未确认（已尝试3次），请现场检查: '+str(last_error))

    def verify(self):
        if not self.stop_confirmed:raise RuntimeError('Motive停止未确认，不能宣告Take保存完成')
        # A nonzero file must be stable and openable without sharing before hashing.
        target=str(self.directory/(self.take_name+'.tak')).replace("'","''")
        code="$target='"+target+"';"+'''
$deadline=[DateTime]::UtcNow.AddSeconds('''+str(float(self.export_timeout))+''');
while([DateTime]::UtcNow -lt $deadline){
 $files=@(); if(Test-Path -LiteralPath $target -PathType Leaf){$files=@(Get-Item -LiteralPath $target)};
 if($files.Count -eq 1 -and $files[0].Length -gt 0){
  $f=$files[0]; $size=$f.Length; $mtime=$f.LastWriteTimeUtc; Start-Sleep -Milliseconds 500; $f.Refresh();
  if($f.Length -eq $size -and $f.LastWriteTimeUtc -eq $mtime){
   $handle=$null;
   try {
    $handle=[IO.File]::Open($f.FullName,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::None);
    $sha=[Security.Cryptography.SHA256]::Create();
    try {$hash=([BitConverter]::ToString($sha.ComputeHash($handle))).Replace('-','').ToLower()} finally {$sha.Dispose()};
    @{path=$f.FullName;size=$handle.Length;sha256=$hash} | ConvertTo-Json -Compress; exit 0
   } catch [IO.IOException] {} finally {if($handle){$handle.Dispose()}}
  }
 }
 Start-Sleep -Milliseconds 500
}; throw "Take file missing, empty, or still being written"'''
        result=json.loads(self._powershell(code,timeout=self.export_timeout+15))
        if PureWindowsPath(result['path'])!=self.directory/(self.take_name+'.tak') or result['size']<=0:
            raise ValueError('Take验证返回了错误文件')
        return {'host':self.windows_host,**result,'storage':'windows','copied_to_ubuntu':False}
