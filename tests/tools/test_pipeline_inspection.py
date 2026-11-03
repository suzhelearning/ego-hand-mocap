import tempfile
from pathlib import Path
import unittest
from mocap_pipeline.inspection import inspect_session, select_run

class InspectionTest(unittest.TestCase):
    def test_no_implicit_latest_run(self):
        self.assertEqual(select_run(['one'],None,'slam'),'one')
        self.assertEqual(select_run(['one','two'],'two','slam'),'two')
        for options in [[],['one','two']]:
            with self.assertRaises(ValueError):select_run(options,None,'slam')
        with self.assertRaises(ValueError):select_run(['one'],'missing','slam')

    def test_missing_session_returns_all_blockers_without_writing(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=inspect_session(Path(tmp),'missing')
            self.assertFalse(r['ready'])
            self.assertGreaterEqual(len([c for c in r['checks'] if not c['ready']]),7)
            self.assertEqual(list(Path(tmp).iterdir()),[])

    def test_path_escape_rejected(self):
        with self.assertRaises(ValueError):inspect_session(Path('/tmp'),'../outside')

    def test_template_has_no_legacy_fetches_or_fixed_session(self):
        s=(Path(__file__).resolve().parents[2]/'src/mocap_pipeline/static/index.html').read_text()
        for old in ["fetch('vision.bin')",'originalV','comparison_modes.json','20261030_182714_db5456']:
            self.assertNotIn(old,s)
        self.assertIn('__SESSION_ID__',s)

if __name__=='__main__':unittest.main()
