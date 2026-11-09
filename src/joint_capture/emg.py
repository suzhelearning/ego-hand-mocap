"""Optional wired EMG0 v1 capture. Host arrival mapping is explicitly coarse."""
from binascii import crc_hqx
import json
from pathlib import Path
import queue
import select
import struct
import threading
import time
import uuid
import h5py
import numpy as np

DEFAULT_PORTS={side:f'/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_44:BD:8D:E1:EB:{suffix}-if00'
               for side,suffix in [('left','70'),('right','6C')]}


class Parser:
    def __init__(self):
        self.buffer=bytearray();self.crc_errors=0;self.protocol_errors=0

    def feed(self,data):
        self.buffer.extend(data);rows=[]
        while len(self.buffer)>=14:
            at=self.buffer.find(b'EMG0')
            if at<0:
                del self.buffer[:-3];break
            if at:del self.buffer[:at]
            if len(self.buffer)<14:break
            _,version,kind,seq,stamp,size=struct.unpack_from('<4sBBHIH',self.buffer)
            if version!=1 or size>256:
                self.protocol_errors+=1;del self.buffer[0];continue
            length=16+size
            if len(self.buffer)<length:break
            packet=bytes(self.buffer[:length])
            if crc_hqx(packet[:-2],0xffff)!=struct.unpack_from('<H',packet,length-2)[0]:
                self.crc_errors+=1;del self.buffer[0];continue
            del self.buffer[:length]
            if kind in {1:68,2:8,5:32,6:24,7:38} and size!={1:68,2:8,5:32,6:24,7:38}[kind]:
                self.protocol_errors+=1;continue
            rows.append(dict(kind=kind,seq=seq,device_us=stamp,payload=packet[14:-2]))
        return rows


def unwrap(raw,high):
    if high is None:return raw,raw
    delta=((raw-(high&0xffffffff)+0x80000000)&0xffffffff)-0x80000000
    value=high+delta
    return value,max(high,value)


def fit_passive(rows):
    result=dict(status='insufficient_data',unknown_fixed_delay=True,physical_sync_valid=False,
                reference_clock='CLOCK_MONOTONIC',method='one latest sample per read; lower arrival envelope per second')
    if len(rows)<4:return result
    rows=np.asarray(rows,dtype=np.int64)
    rows=rows[np.r_[rows[1:,0]!=rows[:-1,0],True]]
    d0=int(rows[0,2]);h0=int(rows[0,1]);x=(rows[:,2]-d0)/1e6;y=(rows[:,1]-h0)/1e9
    if len(x)<4 or x[-1]-x[0]<3:return result
    windows=np.floor(x).astype(int);anchors=[]
    for w in np.unique(windows):
        ix=np.flatnonzero(windows==w);anchors.append(ix[np.argmin((y-x)[ix])])
    if len(anchors)<4:return result
    a,b=np.polyfit(x[anchors],y[anchors],1);res=(y-a*x-b)*1000
    if not np.isfinite(a+b) or a<=0:return result
    result.update(status='passive_coarse',a=float(a),b_relative_s=float(b),device_origin_us=d0,
                  host_origin_ns=h0,apparent_rate_ppm=float((a-1)*1e6),anchors=len(anchors),
                  device_valid_interval_us=[int(rows[0,2]),int(rows[-1,2])],
                  arrival_residual_p95_ms=float(np.percentile(res,95)),
                  formula='host_ns = host_origin_ns + 1e9*(a*(device_us_unwrapped-device_origin_us)/1e6+b_relative_s)')
    return result


def map_passive(stamps,model):
    stamps=np.asarray(stamps,dtype=np.int64)
    values=model['host_origin_ns']+np.rint(1e9*(model['a']*((stamps-model['device_origin_us'])/1e6)+model['b_relative_s'])).astype(np.int64)
    lo,hi=model['device_valid_interval_us']
    return values,(stamps<lo)|(stamps>hi)


class Wristband:
    def __init__(self,side,port,folder):
        self.side=side;self.port=port;self.folder=folder;self.source=None;self.error=None
        self.stop=threading.Event();self.reader=None;self.writer=None;self.q=queue.Queue(maxsize=8192)
        self.last=0.;self.counts={};self.parser=Parser();self.high=None;self.previous={};self.gaps={}
        self.ready=threading.Event();self.session_id=uuid.uuid4().hex;self.invalid=0;self.saturated=0
        self.latest_channels=None;self.last_types={};self.status_drops=[];self.model={};self.summary_data={}

    def start(self):
        import serial
        from serial.tools import list_ports
        matches=[p for p in list_ports.comports() if Path(p.device).resolve()==Path(self.port).resolve()]
        if len(matches)!=1 or (matches[0].vid,matches[0].pid)!=(0x303a,0x1001):
            raise RuntimeError(f'{self.side}: 未找到指定ESP32有线腕带 {self.port}')
        self.identity=matches[0].serial_number
        self.source=serial.Serial(port=None,baudrate=115200,timeout=0,exclusive=True)
        self.source.dtr=False;self.source.rts=False;self.source.port=self.port
        self.source.open();self.source.reset_input_buffer();self.last=time.monotonic()
        self.writer=threading.Thread(target=self._write,name=f'emg-{self.side}-write',daemon=True)
        self.reader=threading.Thread(target=self._read,name=f'emg-{self.side}-read',daemon=True)
        self.writer.start();self.reader.start()

    def _read(self):
        index=0
        try:
            while not self.stop.is_set():
                if not select.select([self.source.fileno()],[],[],.05)[0]:continue
                data=self.source.read(max(1,self.source.in_waiting));stamp=time.monotonic_ns()
                if data:
                    self.q.put_nowait((index,stamp,data));index+=1
        except Exception as exc:self.error=f'串口读取/队列错误: {exc}';self.stop.set()
        finally:
            if self.source:self.source.close()

    def _write(self):
        # Reader stamps before this disk/parse worker; never synthesize per-frame arrival times.
        try:
            with (self.folder/f'{self.side}.bin').open('xb') as raw, (self.folder/f'{self.side}_reads.jsonl').open('x') as log, h5py.File(self.folder/f'{self.side}.h5','x') as h5:
                h5.attrs.update(session_id=self.session_id,device_serial=self.identity,port=self.port,
                    clock_sync='passive_coarse_only',physical_sync_valid=False,complete=False,
                    pc_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
                buffers={};datasets={};timing=[];first_host=None;last_anchor=None
                def flush():
                    for kind,rows in buffers.items():
                        if not rows:continue
                        arr=np.array(rows,dtype=np.int64)
                        if kind not in datasets:
                            ds=h5.create_dataset(kind,shape=(0,arr.shape[1]),maxshape=(None,arr.shape[1]),dtype='i8',chunks=True,compression='gzip')
                            ds.attrs['columns']='read_id,host_read_monotonic_ns,device_us,device_us_unwrapped,sequence,frame_order,flags,'+('ch1..ch16_adc_counts' if kind=='emg' else 'module_ms,ax_ug,ay_ug,az_ug,gx_urad_s,gy_urad_s,gz_urad_s')
                            datasets[kind]=ds
                        ds=datasets[kind];n=len(ds);ds.resize(n+len(arr),axis=0);ds[n:]=arr;rows.clear()
                    h5.flush()
                try:
                    while not self.stop.is_set() or (self.reader and self.reader.is_alive()) or not self.q.empty():
                        try:index,host,data=self.q.get(timeout=.1)
                        except queue.Empty:continue
                        offset=raw.tell();raw.write(data)
                        log.write(json.dumps(dict(read_id=index,host_read_monotonic_ns=host,offset=offset,length=len(data)))+'\n')
                        first_host=host if first_host is None else first_host;anchor=None
                        for order,f in enumerate(self.parser.feed(data)):
                            kind=f['kind'];self.counts[kind]=self.counts.get(kind,0)+1
                            key=1 if kind==2 else kind
                            if key in self.previous:
                                delta=(f['seq']-self.previous[key])&65535
                                if delta==0 or delta>=32768:raise RuntimeError('序号重复/倒退，停止并保留本段；需新建会话')
                                if delta>1:self.gaps[key]=self.gaps.get(key,0)+delta-1
                            self.previous[key]=f['seq']
                            if kind==2:self.status_drops.append(struct.unpack('<II',f['payload'])[1]);continue
                            if kind not in (1,5,6,7):continue
                            unwrapped,self.high=unwrap(f['device_us'],self.high)
                            if kind not in (1,5):continue
                            previous_stamp=self.last_types.get(kind)
                            if previous_stamp is not None and unwrapped<previous_stamp-1000000:raise RuntimeError('设备时间明显回退，停止并新建会话')
                            self.last_types[kind]=unwrapped
                            values=struct.unpack('<I16i' if kind==1 else '<II6i',f['payload'])
                            name='emg' if kind==1 else 'imu'
                            buffers.setdefault(name,[]).append((index,host,f['device_us'],unwrapped,f['seq'],order,*values))
                            if kind==1:
                                self.last=time.monotonic();self.invalid+=bool(values[0]&(1<<31))
                                self.saturated+=any(v in (-8388608,8388607) for v in values[1:]);self.latest_channels=list(values[1:])
                                # Store one lower-envelope candidate per second, bounded independently of sample rate.
                                if host-first_host>1_000_000_000:anchor=(index,host,unwrapped)
                            if 1 in self.counts and 5 in self.counts:self.ready.set()
                        if anchor:
                            window=(host-first_host)//1_000_000_000
                            if last_anchor is None or window!=last_anchor:
                                timing.append(anchor);last_anchor=window
                            elif anchor[1]-anchor[2]*1000<timing[-1][1]-timing[-1][2]*1000:timing[-1]=anchor
                        if sum(map(len,buffers.values()))>=512:flush()
                finally:
                    flush();raw.flush();log.flush()
                flush();raw.flush();log.flush()
                self.model=fit_passive(timing)
                h5.create_dataset('clock_anchor_candidates',data=np.asarray(timing,dtype=np.int64).reshape(-1,3))
                h5.attrs['clock_model_json']=json.dumps(self.model)
                if self.model.get('status')=='passive_coarse':
                    for name,ds in datasets.items():
                        group=h5.create_group(name+'_coarse_alignment')
                        mapped=group.create_dataset('pc_timestamp_ns',shape=(len(ds),),dtype='i8',compression='gzip')
                        extrap=group.create_dataset('clock_extrapolated',shape=(len(ds),),dtype='?',compression='gzip')
                        group.attrs['physical_sync_valid']=False
                        for begin in range(0,len(ds),8192):
                            values,flags=map_passive(ds[begin:begin+8192,3],self.model)
                            mapped[begin:begin+len(values)]=values;extrap[begin:begin+len(values)]=flags
                h5.attrs['complete']=not bool(self.error)
        except Exception as exc:self.error=f'解析/保存错误: {exc}';self.stop.set()
        finally:self.ready.set()

    def check(self):
        if self.error:raise RuntimeError(f'{self.side}: {self.error}')
        if time.monotonic()-self.last>1.5:raise RuntimeError(f'{self.side}: 超过1.5秒没有肌电样本')

    def close(self):
        self.stop.set()
        if self.reader:self.reader.join(timeout=5)
        if self.writer:self.writer.join(timeout=15)
        if (self.reader and self.reader.is_alive()) or (self.writer and self.writer.is_alive()):self.error='采集线程未正常结束'
        if self.source:self.source.close()
        return dict(side=self.side,port=self.port,device_serial=getattr(self,'identity',None),session_id=self.session_id,
                    counts=self.counts,crc_errors=self.parser.crc_errors,protocol_errors=self.parser.protocol_errors,
                    sequence_gaps=self.gaps,invalid_emg_frames=self.invalid,saturated_emg_frames=self.saturated,
                    firmware_drops_first=self.status_drops[0] if self.status_drops else None,
                    firmware_drops_last=self.status_drops[-1] if self.status_drops else None,error=self.error,clock_model=self.model)


class EmgCapture:
    def __init__(self,session,ports):
        self.folder=Path(session)/'emg';self.folder.mkdir();self.workers=[];self.report=None
        if len({str(Path(p).resolve()) for p in ports.values()})!=len(ports):raise ValueError('左右手不能使用同一个串口')
        self.workers=[Wristband(side,port,self.folder) for side,port in ports.items()]

    def start(self):
        for w in self.workers:w.start()
        for w in self.workers:
            if not w.ready.wait(5):raise RuntimeError(f'{w.side}: 未收到EMG及IMU数据')
            w.check()

    def check(self):
        for w in self.workers:w.check()

    def close(self):
        for w in self.workers:w.stop.set()
        self.report=dict(clock_sync='passive_coarse_only',physical_sync_valid=False,devices=[w.close() for w in self.workers])
        (self.folder/'report.json').write_text(json.dumps(self.report,ensure_ascii=False,indent=2))
        return self.report
