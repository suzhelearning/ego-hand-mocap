"""Check separately delivered model files before launching inference."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
errors = []
for item in json.loads((ROOT / 'WEIGHTS_REQUIRED.json').read_text()):
    path = ROOT / item['path']
    if not path.is_file():
        errors.append(f'Missing: {item["path"]}')
        continue
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(chunk)
    if path.stat().st_size != item['bytes'] or digest.hexdigest() != item['sha256']:
        errors.append(f'Checksum mismatch: {item["path"]}')
if errors:
    raise SystemExit('\n'.join(errors))
print('All separately supplied assets verified.')
