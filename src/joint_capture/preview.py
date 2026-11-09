"""Read-only subscribers to Ego's existing H264 preview publishers."""
import io
import struct
import threading
import time


class EgoPreview:
    def __init__(self,host,base_port=5580):
        self.host=host;self.base_port=base_port;self.stop=threading.Event()
        self.lock=threading.Lock();self.frames={};self.errors={};self.threads=[]
        for camera in (0,1):
            t=threading.Thread(target=self._receive,args=(camera,),daemon=True,name=f'ego-preview-{camera}')
            self.threads.append(t);t.start()

    def frame(self,camera):
        if camera not in (0,1):raise ValueError('invalid camera')
        with self.lock:
            item=self.frames.get(camera)
            if item and time.monotonic()-item[0]<1.5:return item[1]
            raise RuntimeError(self.errors.get(camera) or '等待 Ego 实时画面')

    def _receive(self,camera):
        try:
            import av
            import zmq
            with zmq.Context() as ctx:
                sock=ctx.socket(zmq.SUB);sock.setsockopt(zmq.SUBSCRIBE,b'')
                sock.setsockopt(zmq.RCVHWM,16);sock.setsockopt(zmq.LINGER,0)
                sock.connect(f'tcp://{self.host}:{self.base_port+camera}')
                decoder=None;last=None;rendered=0.
                try:
                    while not self.stop.is_set():
                        if not sock.poll(200):continue
                        payload=sock.recv()
                        if len(payload)<14:continue
                        _,seq,key=struct.unpack_from('<QIB',payload)
                        if last is not None and seq!=(last+1)&0xffffffff:decoder=None
                        last=seq
                        if decoder is None:
                            if not key:continue
                            decoder=av.CodecContext.create('h264','r');decoder.thread_count=1
                        try:frames=decoder.decode(av.Packet(payload[13:]))
                        except av.FFmpegError:
                            decoder=None;continue
                        for frame in frames:
                            now=time.monotonic()
                            if now-rendered<.2:continue
                            rendered=now
                            height=max(2,round(frame.height*640/frame.width)//2*2)
                            buffer=io.BytesIO()
                            frame.reformat(width=640,height=height,format='rgb24').to_image().save(buffer,format='JPEG',quality=75)
                            with self.lock:self.frames[camera]=(now,buffer.getvalue());self.errors.pop(camera,None)
                finally:sock.close()
        except Exception as exc:
            with self.lock:self.errors[camera]='Ego 预览异常：'+str(exc)

    def close(self):
        self.stop.set()
        for t in self.threads:t.join(timeout=3)
