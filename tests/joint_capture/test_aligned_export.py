import csv
import importlib.util
import json
from pathlib import Path

import h5py
import numpy as np
import pytest

from joint_capture.h5store import write_version


spec = importlib.util.spec_from_file_location('export_aligned_h5', Path(__file__).resolve().parents[2] / 'tools/export_aligned_h5.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('internal_sync', [False, True])
def test_export_legacy_capture_session(tmp_path, internal_sync):
    session = tmp_path / 'capture_sessions/s1'
    session.mkdir(parents=True)
    # Legacy manifests may reference exported ego data elsewhere in the project.
    ego = tmp_path / 'exported_ego.h5'
    with h5py.File(ego, 'w') as f:
        f['raw'] = [1]
    mocap = session / 'natnet.h5'
    with h5py.File(mocap, 'w') as f:
        f.create_group('rigid_bodies')
        f['frames/publisher_received_time_ns'] = [100]
    (session / 'session.json').write_text(json.dumps({
        'session_id': 's1', 'ego': {'h5': str(ego), 'local_directory': str(tmp_path)},
        'mocap': {'file': str(mocap)},
    }))
    columns = {'mocap_index': [0], 'within_mocap_coverage': [True], 'clock_extrapolated': [False]}
    summary = json.dumps({'cameras': {'RGB_left': {}}, 'source_status': 'complete'})
    if internal_sync:
        def writer(g):
            g['summary_json'] = summary
            g['clock_models_json'] = '{}'
            for key, values in columns.items():
                g['cameras/RGB_left/' + key] = np.asarray(values)
        write_version(session / 'aligned.h5', 'sync', 'v1', writer)
    else:
        sync = tmp_path / 'results/alignment_20261029/s1'
        sync.mkdir(parents=True)
        (sync / 'summary.json').write_text(summary)
        (sync / 'clock_models.json').write_text('{}')
        with (sync / 'RGB_left_nearest.csv').open('w') as handle:
            writer = csv.writer(handle)
            writer.writerow(columns)
            writer.writerow([0, True, False])
    output = module.export(tmp_path, 's1')
    with h5py.File(output) as f:
        assert f['cameras/RGB_left/mocap_received_pc_ns'][:].tolist() == [100]
        assert f.attrs['ego_source_h5'] == str(ego)
        assert f['cameras/RGB_left'].attrs['video_path'] == str(tmp_path / 'video/RGB_left.mp4')
        if internal_sync:
            assert f['processing/alignment/v1'].attrs['complete']
