"""URDF device assets and explicit reference-body frame compatibility."""
import hashlib
import json
from pathlib import Path
import numpy as np
from ego_live.model import load_urdf, configured_installation


def device_inputs(root):
    config_path=root/'config/ego_device_geometry.json'
    config=json.loads(config_path.read_text())
    urdf=root/config['urdf'];model=load_urdf(urdf)
    paths=[config_path,urdf,root/'src/ego_live/model.py',root/'src/ego_live/server.py',Path(__file__)]
    paths += [urdf.parent/v['file'] for v in model['visuals']]
    return config,model,paths


def prepare_device(root,reference,staged):
    config,model,paths=device_inputs(root)
    transform=configured_installation(config,'z')
    # Historical marker-fit frames are not automatically NatNet streamed frames.
    bridge=reference.get('T_B_ego_rigidbody')
    status='verified_reference_bridge' if bridge is not None else 'missing_reference_bridge'
    t=None
    if bridge is not None:
        bridge=np.asarray(bridge,float)
        if bridge.shape!=(4,4) or not np.isfinite(bridge).all() or not np.allclose(bridge[3],[0,0,0,1]) or not np.allclose(bridge[:3,:3].T@bridge[:3,:3],np.eye(3)) or not np.isclose(np.linalg.det(bridge[:3,:3]),1):raise ValueError('Invalid reference-body bridge')
        t=bridge@transform
    for i,v in enumerate(model['visuals']):
        source=root/config['urdf'];source=source.parent/v['file']
        name=f'device_mesh_{i}.stl';(staged/name).write_bytes(source.read_bytes());v['file']=name
    digest=hashlib.sha256()
    for path in paths:digest.update(str(path.relative_to(root)).encode());digest.update(path.read_bytes())
    report=dict(schema=1,model=model,T_ego_rigidbody_base=transform.tolist(),T_B_base=None if t is None else t.tolist(),
                status=status,available=t is not None,source_body=reference['body_source'],config=config,
                inputs_sha256=digest.hexdigest(),note='旧参考B必须有到当前ego_rigidbody的已核实桥接，不能按名字相同直接套用。')
    (staged/'device.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return report
