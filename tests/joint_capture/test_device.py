import json
import pytest
from joint_capture import device_agent


def test_start_refuses_existing_recording_without_claiming_it(tmp_path, monkeypatch):
    state=tmp_path/'owner.json'
    monkeypatch.setattr(device_agent,'control',lambda p:{'recording':{'active':True}})
    with pytest.raises(RuntimeError,match='already'):
        device_agent.record_action('start','ours',state,tmp_path)
    assert not state.exists()


def test_stop_does_not_stop_another_owner(tmp_path, monkeypatch):
    state=tmp_path/'owner.json';state.write_text(json.dumps({'token':'theirs'}))
    calls=[]
    monkeypatch.setattr(device_agent,'control',lambda p:calls.append(p))
    assert device_agent.record_action('stop','ours',state,tmp_path)['owned'] is False
    assert calls==[]


def test_owned_start_stop_is_idempotent(tmp_path, monkeypatch):
    active=False; toggles=[]
    def control(payload):
        nonlocal active
        if payload.get('record_toggle'):
            active=not active;toggles.append(active)
            if active:(tmp_path/'.head_000001.recording').mkdir()
            else:(tmp_path/'.head_000001.recording').rename(tmp_path/'head_000001')
        return {'recording':{'active':active},'status':'ok'}
    monkeypatch.setattr(device_agent,'control',control)
    p=tmp_path/'owner.json'
    result=device_agent.record_action('start','ours',p,tmp_path)
    assert active and result['owned']
    device_agent.record_action('stop','ours',p,tmp_path)
    device_agent.record_action('stop','ours',p,tmp_path)
    assert toggles==[True,False]


def test_export_rejects_parent_paths():
    for name in ['../head_1','/tmp/head_1','head_1/../../etc','head_1;bad']:
        with pytest.raises(ValueError):device_agent.record_name(name)
    assert device_agent.record_name('head_000007')=='head_000007'


def test_replaced_recording_is_not_stopped(tmp_path, monkeypatch):
    active=False
    def control(payload):
        nonlocal active
        if payload.get('record_toggle'):
            active=not active
            if active:(tmp_path/'.head_000001.recording').mkdir()
        return {'recording':{'active':active},'status':'ok'}
    monkeypatch.setattr(device_agent,'control',control)
    p=tmp_path/'owner.json'
    device_agent.record_action('start','ours',p,tmp_path)
    (tmp_path/'.head_000001.recording').rename(tmp_path/'head_000001')
    (tmp_path/'.head_000002.recording').mkdir()
    result=device_agent.record_action('stop','ours',p,tmp_path)
    assert active, 'must not toggle the replacement recording'
    assert result['ownership_lost'] and not result['owned']


def test_premature_device_stop_is_reported(tmp_path, monkeypatch):
    active=False
    def control(payload):
        nonlocal active
        if payload.get('record_toggle'):
            active=not active
            if active:(tmp_path/'.head_000001.recording').mkdir()
        return {'recording':{'active':active},'status':'ok'}
    monkeypatch.setattr(device_agent,'control',control)
    p=tmp_path/'owner.json';device_agent.record_action('start','ours',p,tmp_path)
    active=False
    result=device_agent.record_action('stop','ours',p,tmp_path)
    assert result['premature'] and result['stopped']


def test_failed_pull_cleans_temporary_export(tmp_path, monkeypatch):
    import subprocess
    from joint_capture.ego import EgoController
    ego=EgoController(token='test',serial='fixture')
    actions=[]
    def request(action,**kwargs):
        actions.append(action)
        return {'export_path':'/test/export','temporary_directory':'/test/temp','sha256':{}}
    ego.request=request
    def fail_pull(*args,**kwargs):raise OSError('disk full')
    monkeypatch.setattr(subprocess,'run',fail_pull)
    import pytest
    with pytest.raises(OSError):ego.export(tmp_path/'output')
    assert actions==['export','cleanup']
