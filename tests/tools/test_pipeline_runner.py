import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from mocap_pipeline.runner import run_pipeline

class RunnerTest(unittest.TestCase):
    def test_failure_resumes_without_rerunning_successful_stages(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);session=root/'data/sessions/test';session.mkdir(parents=True)
            (session/'session.json').write_text('{}');profile=root/'profile.json';profile.write_text('{}')
            outputs={};calls=[];fail=[True];revision={'vision':'a','slam':'a','mano':'a','export':'a'}
            def run(stage):
                def adapter(root,session,profile,version,log):
                    calls.append(stage)
                    if stage=='slam' and fail[0]:raise RuntimeError('simulated crash')
                    outputs[(stage,version)]='output-'+version
                return adapter
            def fingerprint(root,session,profile,stage,selected):return revision[stage]
            with patch('mocap_pipeline.runner.fingerprint',side_effect=fingerprint),patch('mocap_pipeline.runner.completed_output',side_effect=lambda s,k,v:outputs.get((k,v))),patch.dict('mocap_pipeline.runner.ADAPTERS',{s:run(s) for s in ['vision','slam','mano']}):
                with self.assertRaisesRegex(RuntimeError,'simulated'):run_pipeline(root,'test',profile,until='mano')
                state=json.loads((session/'pipeline.json').read_text());self.assertEqual(state['stages']['vision']['status'],'complete');self.assertEqual(state['stages']['slam']['status'],'failed')
                fail[0]=False;run_pipeline(root,'test',profile,until='mano')
                self.assertEqual(calls,['vision','slam','slam','mano'])
                run_pipeline(root,'test',profile,until='mano');self.assertEqual(len(calls),4)
                revision['vision']='b';run_pipeline(root,'test',profile,until='mano');self.assertEqual(calls[-1],'vision');self.assertEqual(len(calls),5)
                # Corruption invalidates a completed output even with unchanged inputs.
                state=json.loads((session/'pipeline.json').read_text());outputs[('mano',state['stages']['mano']['version'])]='corrupt'
                run_pipeline(root,'test',profile,until='mano');self.assertEqual(calls[-1],'mano')

    def test_existing_outputs_require_explicit_adoption(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);session=root/'data/sessions/test';session.mkdir(parents=True);(session/'session.json').write_text('{}')
            profile=root/'profile.json';profile.write_text(json.dumps({'existing_versions':{'vision':'old'}}))
            with patch('mocap_pipeline.runner.fingerprint',return_value='a'),patch('mocap_pipeline.runner.completed_output',return_value='digest'),patch.dict('mocap_pipeline.runner.ADAPTERS',{'vision':lambda *args:self.fail('must adopt')}):
                state=run_pipeline(root,'test',profile,adopt_existing=True,until='vision')
                self.assertEqual(state['stages']['vision']['origin'],'adopted_existing')

if __name__=='__main__':unittest.main()

class SkipOnlyMocapHandTest(unittest.TestCase):
    def test_visual_and_slam_still_run_when_mano_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);session=root/'data/sessions/test';session.mkdir(parents=True)
            (session/'session.json').write_text('{}');profile=root/'profile.json'
            profile.write_text(json.dumps({'skip_mocap_hand':True}))
            calls=[]
            with patch('mocap_pipeline.runner.fingerprint',return_value='abc'),patch('mocap_pipeline.runner.completed_output',return_value='digest'),patch.dict('mocap_pipeline.runner.ADAPTERS',{name:(lambda *args,n=name:calls.append(n)) for name in ['vision','slam','mano']}):
                state=run_pipeline(root,'test',profile,until='mano')
            self.assertEqual(calls,['vision','slam'])
            self.assertEqual(state['stages']['mano']['status'],'skipped')
