"""Host-side Ego controller, authenticated export, and file verification."""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
from . import device_agent


class EgoController:
    def __init__(self, token, serial=None, export_timeout=3600):
        if serial is None:
            lines=subprocess.check_output(['adb','devices'],text=True,timeout=5).splitlines()[1:]
            devices=[s.split()[0] for s in lines if len(s.split())==2 and s.split()[1]=='device']
            if len(devices)!=1:raise RuntimeError('需要恰好一个已连接Ego；多设备时请指定 --ego-serial')
            serial=devices[0]
        self.adb=['adb','-s',serial];self.serial=serial;self.token=token
        self.events=[];self.attempted=False
        self.export_timeout=export_timeout

    def request(self,action,**kwargs):
        payload={'action':action,'token':self.token,**kwargs}
        source=Path(device_agent.__file__).read_text()+'\nrun('+repr(payload)+')\n'
        result=subprocess.run([*self.adb,'shell','python3 -c '+shlex.quote(source)],
            capture_output=True,text=True,timeout=(2*self.export_timeout+60) if action=='export' else 20)
        if result.returncode:raise RuntimeError('Ego '+action+' failed: '+result.stderr[-2000:]+result.stdout[-500:])
        value=json.loads(result.stdout)
        if action!='check':self.events.append({'action':action,'result':value})
        return value

    def status(self):return self.request('status')

    def check(self):return self.request('check')

    def start(self):
        self.attempted=True
        return self.request('start')

    def stop(self):
        if self.attempted:return self.request('stop')
        return {'owned':False}

    def export(self,destination):
        metadata=self.request('export',timeout=self.export_timeout);destination=Path(destination)
        try:
            if destination.exists():raise FileExistsError(str(destination))
            destination.parent.mkdir(parents=True,exist_ok=True)
            subprocess.run([*self.adb,'pull',metadata['export_path'],str(destination)],check=True,timeout=self.export_timeout)
            for name,expected in metadata['sha256'].items():
                p=destination/name
                if not p.resolve().is_relative_to(destination.resolve()):raise RuntimeError('Invalid export path')
                h=hashlib.sha256()
                with p.open('rb') as f:
                    for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
                if h.hexdigest()!=expected:raise RuntimeError('Export checksum mismatch: '+name)
            metadata['local_directory']=str(destination.resolve())
            metadata['h5']=str((destination/metadata['h5_name']).resolve())
        finally:
            if metadata.get('temporary_directory'):
                try:self.request('cleanup',path=metadata['temporary_directory'])
                except Exception as e:metadata['temporary_cleanup_warning']=str(e)
        return metadata
