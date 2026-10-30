"""Append-only raw rigid-body HDF5 storage. No temporal interpolation."""
from pathlib import Path
import json
import h5py
import numpy as np

TIMING = ('stamp_camera_mid_exposure', 'stamp_data_received', 'stamp_transmit',
          'prec_timestamp_secs', 'prec_timestamp_frac_secs')


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    temporary.replace(path)


class RigidH5Writer:
    def __init__(self, path, frequency, session_id):
        self.file = h5py.File(path, 'x')
        self.count = 0
        self.valid_bodies = 0
        self.missing_frames = 0
        self.last_frame = None
        self.file.attrs.update(schema='ego-natnet-raw-rigid-v1', session_id=session_id,
            tick_frequency_hz=int(frequency), interpolated=False, quaternion_order='xyzw',
            position_unit='m', pc_time_domain='linux-clock-monotonic',
            mocap_time_domain='Motive server high-resolution clock',
            clock_mapping_status='raw_probes_saved_not_calibrated', complete=False)
        for name, dtype in [('frame_number','i8'), ('motive_timestamp','f8'),
                            ('publisher_received_time_ns','i8')]:
            self._dataset('frames/'+name, (), dtype)
        for name in TIMING:
            self._dataset('frames/natnet_timing/'+name, (), 'i8', -1)
        self.file['frames/natnet_timing'].attrs['tick_frequency_hz'] = int(frequency)
        self.file['frames/publisher_received_time_ns'].attrs['meaning'] = 'PC callback after parsing, NOT exposure'
        self.file.require_group('rigid_bodies')
        raw=self.file.require_group('raw')
        raw['frames']=self.file['frames'];raw['rigid_bodies']=self.file['rigid_bodies']
        self.file.attrs['layout_version']=2

    def _dataset(self, name, shape, dtype, fill=None):
        return self.file.create_dataset(name, shape=(self.count,*shape),
            maxshape=(None,*shape), dtype=dtype, chunks=True, fillvalue=fill)

    def append(self, frames):
        if not frames:
            return
        end = self.count + len(frames)
        for key in ('frame_number','motive_timestamp','publisher_received_time_ns'):
            ds = self.file['frames/'+key]; ds.resize(end, axis=0)
            ds[self.count:] = [f[key] for f in frames]
        for key in TIMING:
            ds = self.file['frames/natnet_timing/'+key]; ds.resize(end, axis=0)
            ds[self.count:] = [f.get('natnet_timing',{}).get(key) if f.get('natnet_timing',{}).get(key) is not None else -1 for f in frames]
        ids = {str(b['id']) for f in frames for b in f['rigid_bodies']}
        for id in ids - set(self.file['rigid_bodies']):
            root = 'rigid_bodies/'+id+'/'
            for key, shape, dtype, fill in [('present',(),'?',False), ('tracking_valid',(),'?',False),
                    ('position',(3,),'f8',np.nan), ('quaternion_xyzw',(4,),'f8',np.nan), ('mean_error',(),'f8',np.nan)]:
                self._dataset(root+key, shape, dtype, fill)
        for id, group in self.file['rigid_bodies'].items():
            selected = [next((b for b in f['rigid_bodies'] if str(b['id'])==id),None) for f in frames]
            for key, ds in group.items():
                ds.resize(end, axis=0)
                if key=='present': values=[b is not None for b in selected]
                elif key=='tracking_valid': values=[bool(b and b[key]) for b in selected]
                else: values=[b[key] if b else np.full(ds.shape[1:] or (),np.nan) for b in selected]
                ds[self.count:] = values
        for frame in frames:
            n=frame['frame_number']
            if self.last_frame is not None:
                self.missing_frames += max(0,n-self.last_frame-1)
            self.last_frame=n
            self.valid_bodies += sum(b['tracking_valid'] for b in frame['rigid_bodies'])
        self.count=end
        self.file.attrs['frames_written']=end
        self.file.flush()

    def close(self, complete=False, names=None):
        if not self.file.id.valid:
            return
        self.file.attrs['complete']=complete
        self.file.attrs['missing_frame_numbers']=self.missing_frames
        for id, group in self.file['rigid_bodies'].items():
            group.attrs['name']=(names or {}).get(int(id),'')
        self.file.flush(); self.file.close()

    def __enter__(self): return self
    def __exit__(self,*args): self.close(complete=args[0] is None)
